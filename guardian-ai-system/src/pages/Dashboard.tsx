import React, { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import { useUser } from '@clerk/clerk-react';
import { api } from '../../convex/_generated/api';
import { useFeederStore } from '../store/useFeederStore';
import { useSafeQuery, useSafeMutation } from '../lib/useSafeConvex';

export const Dashboard: React.FC = () => {
  const navigate = useNavigate();
  const { user } = useUser();
  const ownerId = user?.id || 'demo_owner_id';

  const { isDispensing, dispenseFood, syncFromConvex } = useFeederStore();

  // Fetch real data from Convex safely
  const convexSettings = useSafeQuery(api.deviceSettings.get, { ownerId }, null);
  const feedHistory = useSafeQuery(api.feedHistory.list, { ownerId }, []);
  const aiDetections = useSafeQuery(api.aiRecognition.list, { ownerId }, []);

  const updateSettingsMutation = useSafeMutation(api.deviceSettings.update);

  // Keep store in sync with Convex
  useEffect(() => {
    if (convexSettings) syncFromConvex(convexSettings);
  }, [convexSettings, syncFromConvex]);

  const [isRefilling, setIsRefilling] = useState(false);
  const [refillWeight, setRefillWeight] = useState(2000);

  // Settings & Telemetry from Convex
  const isOnline = !!convexSettings?.online && (Date.now() - (convexSettings?.lastSeen ?? 0) < 35000);
  const feederStatus = isOnline ? (convexSettings?.deviceStatus || 'STANDBY') : 'OFFLINE';

  const portionWeight = convexSettings?.foodPortion ?? 45;
  const initialFood = convexSettings?.initialFoodAmount ?? 2000;
  const estimatedFoodRemaining = convexSettings?.estimatedFoodRemaining ?? 2000;
  const remainingPercentage = Math.round((estimatedFoodRemaining / initialFood) * 100);
  
  const cooldownMinutes = convexSettings?.cooldownMinutes ?? 30;

  const currentTemp = isOnline && convexSettings?.currentTemperature !== undefined 
    ? `${convexSettings.currentTemperature.toFixed(1)}°C` 
    : "N/A";
  const currentHum = isOnline && convexSettings?.currentHumidity !== undefined 
    ? `${convexSettings.currentHumidity.toFixed(0)}%` 
    : "N/A";
  const currentWeight = isOnline && convexSettings?.currentWeight !== undefined 
    ? `${convexSettings.currentWeight.toFixed(1)}g` 
    : "N/A";
  const cameraStreamUrl = convexSettings?.cameraStreamUrl || localStorage.getItem('guardian_custom_camera_url') || '';
    
  const lastSeenStr = convexSettings?.lastSeen 
    ? new Date(convexSettings.lastSeen).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' })
    : 'Never';

  // Last manual/automatic feed
  const successfulFeeds = feedHistory.filter(f => f.completed);
  const lastFeedEvent = successfulFeeds[0];
  const lastFeedTimeStr = lastFeedEvent 
    ? new Date(lastFeedEvent.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    : 'None';

  const [localDetection, setLocalDetection] = useState<any>(() => {
    try {
      const saved = localStorage.getItem('guardian_last_ai_detection');
      return saved ? JSON.parse(saved) : null;
    } catch {
      return null;
    }
  });

  useEffect(() => {
    const syncDetection = () => {
      try {
        const saved = localStorage.getItem('guardian_last_ai_detection');
        setLocalDetection(saved ? JSON.parse(saved) : null);
      } catch {
        setLocalDetection(null);
      }
    };
    window.addEventListener('storage', syncDetection);
    window.addEventListener('guardian-ai-detection-update', syncDetection);
    return () => {
      window.removeEventListener('storage', syncDetection);
      window.removeEventListener('guardian-ai-detection-update', syncDetection);
    };
  }, []);

  // Last AI Detected Companion: prefer the newest cloud or local-camera result.
  const cloudDetection = aiDetections[0] as any;
  const lastDetection = localDetection && (!cloudDetection || localDetection.timestamp > cloudDetection.timestamp)
    ? localDetection
    : cloudDetection;
  const lastDetectedPetName = lastDetection?.recognizedPetName || (lastDetection?.authorized ? "Authorized Pet" : lastDetection ? "Unknown Animal" : "None");
  const lastDetectedPetSpecies = lastDetection?.speciesDetected || '';
  const lastDetectionTimeStr = lastDetection 
    ? new Date(lastDetection.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
    : 'No activity';

  // Dynamic Cooldown remaining (seconds)
  const [cooldownSeconds, setCooldownSeconds] = useState(0);
  const latestFeedTimestamp = feedHistory[0]?.timestamp;

  useEffect(() => {
    if (!latestFeedTimestamp || !convexSettings) {
      setCooldownSeconds(0);
      return;
    }
    const checkCooldown = () => {
      const timePassedMs = Date.now() - latestFeedTimestamp;
      const cooldownMs = cooldownMinutes * 60 * 1000;
      if (timePassedMs < cooldownMs) {
        setCooldownSeconds(Math.ceil((cooldownMs - timePassedMs) / 1000));
      } else {
        setCooldownSeconds(0);
      }
    };

    checkCooldown();
    const interval = setInterval(checkCooldown, 1000);
    return () => clearInterval(interval);
  }, [latestFeedTimestamp, cooldownMinutes, convexSettings]);

  const [dispenseStartWeight, setDispenseStartWeight] = useState(0);

  // Monitor isDispensing to capture start weight
  useEffect(() => {
    if (isDispensing) {
      if (dispenseStartWeight === 0 && convexSettings?.currentWeight !== undefined) {
        setDispenseStartWeight(convexSettings.currentWeight);
      }
    } else {
      setDispenseStartWeight(0);
    }
  }, [isDispensing, convexSettings?.currentWeight, dispenseStartWeight]);

  const dispensedGrams = isDispensing 
    ? Math.max(0, (convexSettings?.currentWeight ?? 0) - dispenseStartWeight)
    : 0;

  // Feed Now manual action
  const handleFeedNow = async () => {
    const activeOwner = ownerId || 'demo_owner_id';
    await dispenseFood(activeOwner, portionWeight);
  };

  const handleRefillSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (refillWeight <= 0 || !ownerId) return;
    try {
      await updateSettingsMutation({
        ownerId,
        initialFoodAmount: refillWeight,
        estimatedFoodRemaining: refillWeight
      });
      setIsRefilling(false);
    } catch (err) {
      console.error('Failed to refill feeder container settings:', err);
    }
  };

  // Convert recent feedings to timeline activities
  const recentActivities = feedHistory.slice(0, 4).map((event) => ({
    id: event._id,
    message: event.completed 
      ? `Dispensed ${event.amountDispensed}g successfully`
      : `Failed to dispense ${event.amountDispensed}g`,
    time: new Date(event.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }),
    type: event.completed ? 'feeding' : 'alert',
    details: {
      statusIcon: event.completed ? 'check_circle' : 'error_outline',
      amount: `${event.amountDispensed}g`,
      statusLabel: event.completed ? 'Dispensed' : 'Failed'
    }
  }));

  return (
    <section className="p-4 md:p-8 space-y-6 max-w-[1440px] mx-auto w-full">
      {/* Header Section */}
      <div className="relative overflow-hidden rounded-[28px] bg-gradient-to-r from-slate-950 via-emerald-950 to-teal-900 px-6 py-7 md:px-9 md:py-8 text-white shadow-xl shadow-emerald-950/10">
        <div className="absolute -top-20 right-16 w-52 h-52 rounded-full bg-emerald-300/10 blur-3xl"></div>
        <div className="absolute -bottom-24 -right-8 w-64 h-64 rounded-full border-[32px] border-white/[0.04]"></div>
        <div className="relative flex flex-col md:flex-row md:items-center justify-between gap-6">
          <div className="flex items-center gap-4">
            <div className="w-14 h-14 rounded-2xl bg-white/10 border border-white/10 backdrop-blur flex items-center justify-center shadow-inner">
              <span className="material-symbols-outlined text-emerald-300 text-[30px]">pets</span>
            </div>
            <div>
              <div className="flex items-center gap-2 mb-1.5">
                <span className="w-2 h-2 rounded-full bg-emerald-300 animate-pulse"></span>
                <p className="text-[10px] uppercase tracking-[0.24em] font-bold text-emerald-200">Guardian AI Command Center</p>
              </div>
              <h3 className="font-black text-2xl md:text-3xl tracking-tight">Everything your companion needs.</h3>
              <p className="text-sm text-slate-300 mt-1.5">Live recognition, feeding and wellbeing in one calm overview.</p>
            </div>
          </div>
          <button 
            onClick={() => navigate('/settings')}
            className="self-start md:self-auto px-5 py-3 rounded-xl bg-white text-emerald-900 font-bold text-xs hover:bg-emerald-50 hover:shadow-lg transition-all duration-200 active:scale-95 flex items-center gap-2 cursor-pointer shrink-0"
          >
            <span className="material-symbols-outlined text-base">tune</span>
            Manage feeder
          </button>
        </div>
      </div>

      {/* Bento Grid */}
      <div className="bento-grid">
        {/* Large Status Card (Span 12) - System Status */}
        <div className="col-span-12 glass-card p-8 flex flex-col justify-between relative overflow-hidden bg-white">
          <div className="absolute -top-24 -right-24 w-64 h-64 bg-primary/5 rounded-full blur-3xl"></div>
          
          <div className="relative z-10">
            <div className="flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 mb-8">
              <div className="flex items-center gap-3">
                <div className="w-12 h-12 rounded-2xl bg-primary/10 flex items-center justify-center">
                  <span className="material-symbols-outlined text-primary" style={{ fontVariationSettings: "'FILL' 1" }}>
                    settings_input_component
                  </span>
                </div>
                <div>
                  <h4 className="font-bold text-lg text-on-surface">System Status</h4>
                  <div className="flex flex-wrap items-center gap-2 mt-0.5">
                    <p className={`text-xs flex items-center gap-1 font-bold uppercase ${
                      feederStatus === 'FEEDING' ? 'text-primary' :
                      feederStatus === 'STANDBY' ? 'text-green-600' :
                      feederStatus === 'COOLDOWN' ? 'text-blue-600' :
                      feederStatus === 'BOOTING' ? 'text-amber-500' :
                      feederStatus === 'OFFLINE' ? 'text-gray-500' :
                      'text-error'
                    }`}>
                      <span className={`w-2 h-2 rounded-full ${
                        feederStatus === 'FEEDING' ? 'bg-primary animate-pulse' :
                        feederStatus === 'STANDBY' ? 'bg-green-600' :
                        feederStatus === 'COOLDOWN' ? 'bg-blue-600 animate-pulse' :
                        feederStatus === 'BOOTING' ? 'bg-amber-500 animate-pulse' :
                        feederStatus === 'OFFLINE' ? 'bg-gray-500' :
                        'bg-error'
                      }`}></span>
                      {feederStatus}
                    </p>
                  </div>
                </div>
              </div>
              <div className="text-left sm:text-right">
                <p className="text-[10px] font-bold text-on-surface-variant uppercase tracking-widest">Last Feeding</p>
                <p className="font-bold text-2xl text-on-surface">{lastFeedTimeStr}</p>
              </div>
            </div>

            {/* Metrics Grid */}
            <div className="grid grid-cols-2 md:grid-cols-3 lg:grid-cols-5 gap-6">
              <div className="space-y-1">
                <p className="text-xs text-on-surface-variant font-medium">Food Portion</p>
                <p className="font-bold text-lg text-on-surface">{portionWeight}g</p>
              </div>
              <div className="space-y-1 col-span-1 md:col-span-1">
                <div className="flex justify-between items-center mr-2">
                  <p className="text-xs text-on-surface-variant font-medium">Remaining Food</p>
                  <button 
                    onClick={() => setIsRefilling(true)} 
                    className="text-[10px] text-primary font-bold hover:underline cursor-pointer"
                  >
                    Refill
                  </button>
                </div>
                <div className="flex items-end gap-2">
                  <p className="font-bold text-lg text-on-surface">
                    {estimatedFoodRemaining}g{' '}
                    <span className="text-xs text-on-surface-variant font-normal">({remainingPercentage}%)</span>
                  </p>
                </div>
                <div className="w-full h-1.5 bg-outline-variant/30 rounded-full overflow-hidden mt-1.5">
                  <div 
                    className="h-full bg-primary rounded-full transition-all duration-500" 
                    style={{ width: `${remainingPercentage}%` }}
                  ></div>
                </div>
              </div>
              <div className="space-y-1">
                <p className="text-xs text-on-surface-variant font-medium">Current Weight</p>
                <p className="font-bold text-lg text-on-surface">{currentWeight}</p>
              </div>
              <div className="space-y-1">
                <p className="text-xs text-on-surface-variant font-medium">Temp / Humidity</p>
                <p className="font-bold text-lg text-on-surface">{currentTemp} / {currentHum}</p>
              </div>
              <div className="space-y-1">
                <p className="text-xs text-on-surface-variant font-medium">Last Seen</p>
                <p className="font-bold text-xs text-on-surface">{lastSeenStr}</p>
              </div>
            </div>
          </div>

          <div className="mt-8 pt-8 border-t border-outline-variant/30 flex flex-col sm:flex-row items-start sm:items-center justify-between gap-4 relative z-10">
            <div className="flex items-center gap-4">
              <div className="w-14 h-14 rounded-full border-2 border-primary/20 p-1 bg-surface-container flex items-center justify-center overflow-hidden">
                {lastDetection?.imageUrl ? (
                  <img
                    className="w-full h-full rounded-full object-cover"
                    src={lastDetection.imageUrl}
                    alt={lastDetectedPetName}
                  />
                ) : (
                  <span className="material-symbols-outlined text-on-surface-variant text-2xl">pets</span>
                )}
              </div>
              <div>
                <p className="text-[10px] uppercase tracking-widest font-bold text-on-surface-variant">Last Detected Companion</p>
                <p className="text-base font-bold text-on-surface mt-0.5">{lastDetectedPetName}</p>
                {lastDetectedPetSpecies && lastDetectedPetSpecies !== lastDetectedPetName && (
                  <p className="text-xs text-on-surface-variant mt-0.5">{lastDetectedPetSpecies}</p>
                )}
              </div>
            </div>
            <div className="flex gap-2">
              <span className="px-3 py-1 bg-surface-container text-on-surface-variant rounded-full text-xs font-medium">
                {lastDetectionTimeStr}
              </span>
              <span className={`px-3 py-1 rounded-full text-xs font-bold ${
                lastDetection?.authorized 
                  ? 'bg-primary/10 text-primary' 
                  : lastDetection 
                    ? 'bg-error/10 text-error' 
                    : 'bg-surface-variant text-on-surface-variant'
              }`}>
                {lastDetection?.authorized ? "Verified AI" : lastDetection ? "Unrecognized" : "No Activity"}
              </span>
            </div>
          </div>
        </div>

        {/* Live Camera Section (Span 12) */}
        <div className="col-span-12 glass-card overflow-hidden bg-white">
          <div className="p-6 border-b border-outline-variant flex items-center justify-between">
            <div className="flex items-center gap-3">
              <span className="w-3 h-3 bg-error rounded-full animate-ping"></span>
              <h4 className="font-bold text-lg text-on-surface">Live Camera Preview</h4>
              <span className="px-2 py-0.5 bg-surface-variant text-on-surface-variant rounded text-[10px] font-bold uppercase tracking-wider">
                ESP32-CAM Stream
              </span>
            </div>
            <div className="flex items-center gap-2">
              <button 
                onClick={() => navigate('/camera')}
                className="p-2 rounded-full hover:bg-surface-container-highest transition-colors active:scale-95 cursor-pointer"
              >
                <span className="material-symbols-outlined">fullscreen</span>
              </button>
            </div>
          </div>
          <div className="flex flex-col lg:flex-row">
            {/* Stream View */}
            <div className="lg:w-2/3 aspect-video bg-black relative group cursor-pointer flex items-center justify-center" onClick={() => navigate('/camera')}>
              {cameraStreamUrl ? (
                <img 
                  className="absolute inset-0 w-full h-full object-cover opacity-80 group-hover:opacity-100 transition-all duration-300"
                  src={cameraStreamUrl}
                  alt="ESP32 Live Feed"
                  onError={(e) => {
                    e.currentTarget.style.display = 'none';
                  }}
                />
              ) : (
                <div className="text-center text-on-surface-variant flex flex-col items-center gap-2">
                  <span className="material-symbols-outlined text-4xl opacity-50">videocam_off</span>
                  <p className="text-sm font-semibold">Camera Offline</p>
                  <p className="text-[10px] opacity-75">No Camera Stream URL configured in settings.</p>
                </div>
              )}
              {cameraStreamUrl && (
                <div className="absolute top-4 left-4 bg-black/40 backdrop-blur-md text-white px-3 py-1 rounded-full text-xs flex items-center gap-2">
                  <span className="material-symbols-outlined text-xs">videocam</span>
                  Live Stream
                </div>
              )}
            </div>
            {/* Controls & AI Summary */}
            <div className="lg:w-1/3 p-6 flex flex-col justify-between gap-6 bg-surface-container-low border-l border-outline-variant/20">
              {/* AI Recognition Summary Panel */}
              <div className="relative overflow-hidden bg-gradient-to-br from-slate-950 via-slate-900 to-emerald-950 p-5 rounded-3xl border border-emerald-400/15 text-white shadow-xl shadow-slate-900/10">
                <div className="absolute -right-10 -top-10 w-32 h-32 rounded-full bg-emerald-400/10 blur-2xl"></div>
                <div className="relative space-y-4">
                  <div className="flex items-center justify-between gap-3">
                    <div className="flex items-center gap-2.5">
                      <div className="w-9 h-9 rounded-xl bg-emerald-400/15 flex items-center justify-center border border-emerald-300/15">
                        <span className="material-symbols-outlined text-emerald-300 text-xl">neurology</span>
                      </div>
                      <div>
                        <p className="text-[10px] uppercase tracking-[0.18em] text-slate-400 font-bold">Local AI</p>
                        <p className="text-sm font-bold">Recognition Status</p>
                      </div>
                    </div>
                    <span className={`px-2.5 py-1 font-extrabold text-[9px] tracking-wider rounded-full border ${
                      lastDetection?.authorized
                        ? 'bg-emerald-400/15 text-emerald-300 border-emerald-300/20'
                        : lastDetection
                          ? 'bg-rose-400/15 text-rose-300 border-rose-300/20'
                          : 'bg-slate-400/10 text-slate-300 border-slate-300/15'
                    }`}>
                      {lastDetection?.petMatch ?? (lastDetection?.authorized ? 'AUTHORIZED' : lastDetection ? 'UNAUTHORIZED' : 'NO SCAN')}
                    </span>
                  </div>

                  <div className="grid grid-cols-[1fr_auto] gap-3 items-end bg-white/[0.06] border border-white/[0.07] rounded-2xl p-4">
                    <div className="min-w-0">
                      <p className="text-[9px] font-bold text-slate-400 uppercase tracking-widest">Detected subject</p>
                      <p className="font-bold text-base truncate mt-1">{lastDetection ? lastDetectedPetName : 'Awaiting first scan'}</p>
                      {lastDetection && lastDetectedPetSpecies !== lastDetectedPetName && (
                        <p className="text-[10px] text-slate-400 truncate mt-0.5">{lastDetectedPetSpecies}</p>
                      )}
                    </div>
                    <div className="text-right">
                      <p className="text-[9px] font-bold text-slate-400 uppercase tracking-widest">Confidence</p>
                      <p className="font-black text-2xl text-emerald-300 mt-0.5">
                        {lastDetection ? `${Math.round((lastDetection.confidence ?? 0) <= 1 ? (lastDetection.confidence ?? 0) * 100 : lastDetection.confidence)}%` : '—'}
                      </p>
                    </div>
                  </div>

                  <div className="flex justify-between items-center text-[10px] text-slate-400">
                    <span>Model: <strong className="text-slate-200">{lastDetection?.modelName || 'Guardian Visual Identity'}</strong></span>
                    <span>Latency: <strong className="text-slate-200">{Number.isFinite(lastDetection?.inferenceTimeMs) ? `${lastDetection.inferenceTimeMs}ms` : '—'}</strong></span>
                  </div>
                </div>
              </div>

              <div className="w-full flex flex-col gap-3">
                <button 
                  disabled={isDispensing}
                  onClick={handleFeedNow}
                  className={`w-full py-3.5 rounded-xl font-bold text-sm hover:shadow-lg active:scale-98 transition-all flex items-center justify-center gap-2 cursor-pointer ${
                    isDispensing
                      ? 'bg-primary/50 text-white cursor-not-allowed' 
                      : 'bg-primary text-on-primary hover:bg-primary-container shadow-md'
                  }`}
                >
                  <span className="material-symbols-outlined">restaurant</span>
                  {isDispensing ? `Dispensing (${dispensedGrams.toFixed(1)}g)...` : 'Feed Now'}
                </button>
                {cooldownSeconds > 0 && !isDispensing && (
                  <div className="w-full mt-1 p-3 bg-amber-50 border border-amber-200/50 rounded-xl flex items-center gap-2">
                    <span className="material-symbols-outlined text-amber-600 text-lg animate-pulse">info</span>
                    <div className="text-left">
                      <p className="text-[11px] font-bold text-amber-800">Cooldown Active</p>
                      <p className="text-[10px] text-amber-700">
                        Remaining lockout: {Math.floor(cooldownSeconds / 60)}m {cooldownSeconds % 60}s
                      </p>
                    </div>
                  </div>
                )}
              </div>
            </div>
          </div>
        </div>

        {/* Recent Activity Section (Span 12) */}
        <div className="col-span-12 glass-card p-8 flex flex-col bg-white">
          <div className="flex items-center justify-between mb-8">
            <h4 className="font-bold text-lg text-on-surface">Recent Activity</h4>
            <button 
              onClick={() => navigate('/history')}
              className="text-primary font-bold text-sm hover:underline cursor-pointer"
            >
              View Detailed History
            </button>
          </div>
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
            {recentActivities.length > 0 ? (
              recentActivities.map((event) => (
                <div 
                  key={event.id}
                  className="flex gap-4 p-4 rounded-2xl bg-[#f8f9ff] border border-outline-variant/30 hover:border-primary/20 transition-all duration-200"
                >
                  <div className={`w-10 h-10 rounded-full flex items-center justify-center shrink-0 shadow-sm ${
                    event.type === 'feeding' 
                      ? 'bg-primary text-white' 
                      : 'bg-error text-white'
                  }`}>
                    <span className="material-symbols-outlined text-lg" style={{ fontVariationSettings: "'FILL' 1" }}>
                      {event.details.statusIcon}
                    </span>
                  </div>
                  <div>
                    <p className="font-bold text-xs text-on-surface">{event.message}</p>
                    <p className="text-[11px] text-on-surface-variant mt-0.5">
                      {event.time} • {event.details.amount}
                    </p>
                  </div>
                </div>
              ))
            ) : (
              <p className="text-sm text-on-surface-variant italic col-span-4 text-center py-4">No recent feeding activity.</p>
            )}
          </div>
        </div>
      </div>

      {/* Refill Container Modal */}
      {isRefilling && (
        <div className="fixed inset-0 bg-black/50 z-50 flex items-center justify-center p-4 backdrop-blur-xs">
          <div className="bg-white rounded-3xl border border-outline-variant p-8 max-w-sm w-full space-y-6 shadow-2xl">
            <div>
              <h4 className="font-bold text-lg text-on-surface">Refill Food Container</h4>
              <p className="text-xs text-on-surface-variant mt-1">Configure the new initial amount of food added to the container.</p>
            </div>
            <form onSubmit={handleRefillSubmit} className="space-y-4">
              <div>
                <label className="text-xs font-bold text-on-surface-variant uppercase tracking-wider block mb-1.5">Initial Capacity (grams)</label>
                <input 
                  type="number"
                  required
                  value={refillWeight}
                  onChange={(e) => setRefillWeight(Number(e.target.value))}
                  className="w-full px-4 py-3 rounded-xl border border-outline-variant/60 focus:outline-none focus:border-primary text-sm font-semibold"
                />
              </div>
              <div className="flex gap-3 justify-end pt-2">
                <button 
                  type="button"
                  onClick={() => setIsRefilling(false)}
                  className="px-5 py-2.5 bg-surface-container hover:bg-surface-container-high rounded-xl font-bold text-xs text-on-surface-variant transition-colors cursor-pointer"
                >
                  Cancel
                </button>
                <button 
                  type="submit"
                  className="px-5 py-2.5 bg-primary hover:bg-primary-container text-white rounded-xl font-bold text-xs transition-colors cursor-pointer"
                >
                  Confirm Refill
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </section>
  );
};

export default Dashboard;
