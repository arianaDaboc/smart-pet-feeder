import React, { useState, useEffect } from 'react';
import { useUser } from '@clerk/clerk-react';
import { api } from '../../convex/_generated/api';
import { useFeederStore } from '../store/useFeederStore';
import { useSafeQuery, useSafeMutation } from '../lib/useSafeConvex';

export const Settings: React.FC = () => {
  const { settings } = useFeederStore();
  const { user } = useUser();
  const ownerId = user?.id || '';

  const updateSettingsMutation = useSafeMutation(api.deviceSettings.update);

  const [activeTab, setActiveTab] = useState<'portion' | 'cooldown' | 'temp' | 'refill' | 'wifi' | 'info' | 'notifications' | 'ai'>('portion');

  const [tempPortion, setTempPortion] = useState(settings.portionWeight);
  const [tempMaxTemp, setTempMaxTemp] = useState(settings.maxTemperature);
  const [refillWeight, setRefillWeight] = useState(settings.initialFoodAdded);
  const [streamUrl, setStreamUrl] = useState(settings.cameraStreamUrl);
  const [wifiSsidInput, setWifiSsidInput] = useState(settings.wifiSsid);
  const [tempCooldown, setTempCooldown] = useState(settings.cooldownMinutes);
  const [cooldownDirty, setCooldownDirty] = useState(false);
  const [geminiApiKey, setGeminiApiKey] = useState(() =>
    localStorage.getItem('guardian_feeder_gemini_api_key') || localStorage.getItem('guardian_gemini_api_key') || ''
  );
  const [qwenApiKey, setQwenApiKey] = useState(() =>
    localStorage.getItem('guardian_qwen_api_key') || localStorage.getItem('guardian_openrouter_api_key') || ''
  );

  useEffect(() => {
    localStorage.setItem('guardian_feeder_gemini_api_key', geminiApiKey);
    localStorage.setItem('guardian_gemini_api_key', geminiApiKey);
  }, [geminiApiKey]);

  useEffect(() => {
    localStorage.setItem('guardian_qwen_api_key', qwenApiKey);
    localStorage.setItem('guardian_openrouter_api_key', qwenApiKey);
  }, [qwenApiKey]);

  const convexSettings = useSafeQuery(api.deviceSettings.get, { ownerId }, null);

  const [simulateHardware] = useState<boolean>(false);

  const isDeviceOnline = simulateHardware || (convexSettings?.online === true && (convexSettings?.lastSeen ? (Date.now() - convexSettings.lastSeen < 20000) : false));

  const [calStep, setCalStep] = useState<1 | 2 | 3>(1);
  const [refWeightInput, setRefWeightInput] = useState<number>(100);
  const [calStatus, setCalStatus] = useState<string>('');
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [prevPendingCommand, setPrevPendingCommand] = useState<string | undefined>('');
  const [isCalibrationOpen, setIsCalibrationOpen] = useState(false);
  const [saveNotice, setSaveNotice] = useState<'success' | 'error' | null>(null);

  useEffect(() => {
    if (!isDeviceOnline && isLoading) {
      setIsLoading(false);
      setCalStatus('Device went offline. Calibration aborted.');
      setCalStep(1);
    }
  }, [isDeviceOnline, isLoading]);

  useEffect(() => {
    if (!simulateHardware || !isLoading || !convexSettings?.pendingCommand) return;

    const timer = setTimeout(async () => {
      try {
        if (convexSettings.pendingCommand === "CMD:TARE") {
          await updateSettingsMutation({
            ownerId,
            pendingCommand: "",
          });
        } else if (convexSettings.pendingCommand === "CMD:GET_RAW") {
          await updateSettingsMutation({
            ownerId,
            pendingCommand: "",
            rawScaleValue: 34215,
          });
        } else if (convexSettings.pendingCommand?.startsWith("CMD:CALIBRATE:")) {
          const parts = convexSettings.pendingCommand.split(":");
          const ref = Number(parts[2]) || 100;
          await updateSettingsMutation({
            ownerId,
            pendingCommand: "",
            calibrationFactor: 34215 / ref,
          });
        }
      } catch (err) {
        console.error("Simulation error:", err);
      }
    }, 1500);

    return () => clearTimeout(timer);
  }, [simulateHardware, isLoading, convexSettings?.pendingCommand, ownerId]);

  useEffect(() => {
    if (!convexSettings) return;

    if (calStep === 1 && isLoading) {
      if (!convexSettings.pendingCommand) {
        setIsLoading(false);
        setCalStatus('Test dispense completed! Weigh the food and click Continue.');
        setCalStep(2);
      }
    }

    if (calStep === 2 && isLoading) {
      if (!convexSettings.pendingCommand) {
        setIsLoading(false);
      }
    }

    if (calStep === 3 && isLoading) {
      if (convexSettings.calibrationFactor !== undefined || !convexSettings.pendingCommand) {
        setIsLoading(false);
        setCalStatus(`Flow rate saved successfully!`);
      }
    }

    setPrevPendingCommand(convexSettings.pendingCommand);
  }, [convexSettings?.pendingCommand, convexSettings?.calibrationFactor, calStep, isLoading, prevPendingCommand]);

  useEffect(() => {
    setTempPortion(settings.portionWeight);
    setTempMaxTemp(settings.maxTemperature);
    setRefillWeight(settings.initialFoodAdded);
    setStreamUrl(settings.cameraStreamUrl);
    setWifiSsidInput(settings.wifiSsid);
    if (!cooldownDirty) setTempCooldown(settings.cooldownMinutes);
  }, [settings, cooldownDirty]);

  useEffect(() => {
    if (cooldownDirty && Number(convexSettings?.cooldownMinutes) === tempCooldown) {
      setCooldownDirty(false);
    }
  }, [convexSettings?.cooldownMinutes, cooldownDirty, tempCooldown]);

  const handleSaveChanges = async () => {
    if (!ownerId) return;
    const normalizedCooldown = Math.max(1, Math.min(720, Math.round(tempCooldown || 1)));
    setTempCooldown(normalizedCooldown);
    try {
      await updateSettingsMutation({
        ownerId,
        foodPortion: tempPortion,
        maximumTemperature: tempMaxTemp,
        cameraStreamUrl: streamUrl,
        wifiSSID: wifiSsidInput,
        cooldownMinutes: normalizedCooldown
      });
      setSaveNotice('success');
      setIsCalibrationOpen(false);
      window.setTimeout(() => setSaveNotice(null), 3500);
    } catch (err) {
      console.error('Failed to save settings:', err);
      setSaveNotice('error');
    }
  };

  const handleRefillSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (refillWeight <= 0 || !ownerId) return;
    try {
      await updateSettingsMutation({
        ownerId,
        initialFoodAmount: refillWeight,
        estimatedFoodRemaining: refillWeight,
        foodContainerCapacity: refillWeight,
      });
      alert(`Feeder successfully refilled. Initial capacity set to ${refillWeight}g.`);
    } catch (err) {
      console.error('Failed to refill:', err);
    }
  };

  const toggleNotificationSetting = async (key: 'feedingSuccessful' | 'lowFoodWarning' | 'criticalOverheat') => {
    if (!ownerId) return;
    const newNotifications = {
      feedingSuccessful: settings.notificationsEnabled.feedingSuccessful,
      lowFoodWarning: settings.notificationsEnabled.lowFoodWarning,
      criticalOverheat: settings.notificationsEnabled.criticalOverheat,
      [key]: !settings.notificationsEnabled[key]
    };
    try {
      await updateSettingsMutation({
        ownerId,
        notificationsEnabled: newNotifications
      });
    } catch (err) {
      console.error('Failed to update notifications settings:', err);
    }
  };

  return (
    <section className="p-4 md:p-8 space-y-6 max-w-[1440px] mx-auto w-full min-h-screen">

      <div className="flex justify-between items-end">
        <div>
          <h3 className="font-bold text-3xl text-on-surface tracking-tight">Device Configuration</h3>
          <p className="text-sm text-on-surface-variant">Manage the parameters of your Guardian AI device.</p>
        </div>
      </div>

        <div className="flex flex-col md:flex-row gap-8">

          <nav className="w-full md:w-64 space-y-2 shrink-0">
            <button
              onClick={() => setActiveTab('portion')}
              className={`w-full flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 text-xs font-bold ${
                activeTab === 'portion' ? 'bg-white shadow-sm border border-primary/20 text-primary' : 'text-on-surface-variant hover:bg-white/50'
              }`}
            >
              <span className="material-symbols-outlined">scale</span>
              <span>Food Portion</span>
            </button>
            <button
              onClick={() => setActiveTab('cooldown')}
              className={`w-full flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 text-xs font-bold ${
                activeTab === 'cooldown' ? 'bg-white shadow-sm border border-primary/20 text-primary' : 'text-on-surface-variant hover:bg-white/50'
              }`}
            >
              <span className="material-symbols-outlined">timer</span>
              <span>Cooldown Duration</span>
            </button>
            <button
              onClick={() => setActiveTab('temp')}
              className={`w-full flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 text-xs font-bold ${
                activeTab === 'temp' ? 'bg-white shadow-sm border border-primary/20 text-primary' : 'text-on-surface-variant hover:bg-white/50'
              }`}
            >
              <span className="material-symbols-outlined">device_thermostat</span>
              <span>Maximum Temperature</span>
            </button>
            <button
              onClick={() => setActiveTab('refill')}
              className={`w-full flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 text-xs font-bold ${
                activeTab === 'refill' ? 'bg-white shadow-sm border border-primary/20 text-primary' : 'text-on-surface-variant hover:bg-white/50'
              }`}
            >
              <span className="material-symbols-outlined">inventory_2</span>
              <span>Initial Food Added</span>
            </button>
            <button
              onClick={() => setActiveTab('wifi')}
              className={`w-full flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 text-xs font-bold ${
                activeTab === 'wifi' ? 'bg-white shadow-sm border border-primary/20 text-primary' : 'text-on-surface-variant hover:bg-white/50'
              }`}
            >
              <span className="material-symbols-outlined">wifi</span>
              <span>Wi-Fi Settings</span>
            </button>
            <button
              onClick={() => setActiveTab('ai')}
              className={`w-full flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 text-xs font-bold ${
                activeTab === 'ai' ? 'bg-white shadow-sm border border-primary/20 text-primary' : 'text-on-surface-variant hover:bg-white/50'
              }`}
            >
              <span className="material-symbols-outlined">psychology</span>
              <span>AI Settings</span>
            </button>
            <button
              onClick={() => setActiveTab('info')}
              className={`w-full flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 text-xs font-bold ${
                activeTab === 'info' ? 'bg-white shadow-sm border border-primary/20 text-primary' : 'text-on-surface-variant hover:bg-white/50'
              }`}
            >
              <span className="material-symbols-outlined">info</span>
              <span>Device Information</span>
            </button>
            <button
              onClick={() => setActiveTab('notifications')}
              className={`w-full flex items-center gap-3 px-4 py-3 rounded-xl transition-all duration-200 text-xs font-bold ${
                activeTab === 'notifications' ? 'bg-white shadow-sm border border-primary/20 text-primary' : 'text-on-surface-variant hover:bg-white/50'
              }`}
            >
              <span className="material-symbols-outlined">notifications_active</span>
              <span>Notification Preferences</span>
            </button>
          </nav>

          <div className="flex-1 bg-white rounded-3xl border border-outline-variant/30 p-8 shadow-sm">

            {activeTab === 'portion' && (
              <div className="space-y-8">
                <div className="flex items-start justify-between">
                  <div>
                    <h3 className="font-bold text-lg text-on-surface">Portion Control</h3>
                    <p className="text-xs text-on-surface-variant mt-1">Define the exact amount of kibble dispensed per feeding event (grams).</p>
                  </div>
                  <span className="px-3 py-1 bg-primary/10 text-primary text-[9px] font-bold rounded-full uppercase tracking-wider">Calibration Ready</span>
                </div>
                <div className="p-6 bg-surface-bright rounded-2xl border border-outline-variant/30">
                  <label className="text-sm font-bold block mb-4">Dispense Weight (Grams)</label>
                  <input
                    type="range"
                    min="10"
                    max="250"
                    value={tempPortion}
                    onChange={async (e) => {
                      const newPortion = Number(e.target.value);
                      setTempPortion(newPortion);
                      if (ownerId) {
                        try {
                          await updateSettingsMutation({ ownerId, foodPortion: newPortion });
                        } catch (err) {
                          console.error("Failed to update portion:", err);
                        }
                      }
                    }}
                    className="w-full h-2 bg-surface-container rounded-lg appearance-none cursor-pointer accent-primary"
                  />
                  <div className="flex justify-between mt-3 text-xs text-on-surface-variant">
                    <span>10g</span>
                    <span className="text-primary font-bold">{tempPortion}g (Current)</span>
                    <span>250g</span>
                  </div>
                </div>

                <div className="pt-6 border-t border-outline-variant/30 space-y-4">
                  <button
                    type="button"
                    onClick={() => setIsCalibrationOpen((open) => !open)}
                    className={`w-full flex items-center justify-between gap-4 rounded-2xl border px-4 py-3 text-left transition-all ${
                      isCalibrationOpen
                        ? 'bg-primary/[0.06] border-primary/25 shadow-sm'
                        : 'bg-surface-bright border-outline-variant/30 hover:border-primary/30 hover:bg-primary/[0.03]'
                    }`}
                  >
                    <div className="flex items-center gap-3 min-w-0">
                      <span className="w-9 h-9 rounded-xl bg-primary/10 text-primary flex items-center justify-center shrink-0">
                        <span className="material-symbols-outlined text-[18px]">tune</span>
                      </span>
                      <div className="min-w-0">
                        <div className="flex flex-wrap items-center gap-2">
                          <h4 className="font-bold text-sm text-on-surface">Flow Rate Calibration</h4>
                          <span className="px-2 py-0.5 bg-secondary/10 text-secondary text-[9px] font-bold rounded-full">
                            {convexSettings?.calibrationFactor !== undefined ? `${convexSettings.calibrationFactor.toFixed(1)} g/s` : 'Default 5.0 g/s'}
                          </span>
                        </div>
                        <p className="text-[10px] text-on-surface-variant mt-0.5 truncate">
                          {isCalibrationOpen ? 'Calibration wizard open' : 'Open calibration wizard'}
                        </p>
                      </div>
                    </div>
                    <span className={`material-symbols-outlined text-on-surface-variant text-[20px] transition-transform ${isCalibrationOpen ? 'rotate-180' : ''}`}>expand_more</span>
                  </button>

                  {isCalibrationOpen && (
                  <div className="bg-surface-bright border border-outline-variant/30 rounded-2xl p-6 space-y-6 animate-in fade-in duration-200">

                    {!isDeviceOnline && (
                      <div className="p-4 bg-red-50 border border-red-200/50 rounded-xl flex items-start gap-3 text-red-800 max-w-xl mx-auto">
                        <span className="material-symbols-outlined text-base mt-0.5 text-red-600">error</span>
                        <div className="space-y-1">
                          <h5 className="font-bold text-xs text-red-900">Device Offline</h5>
                          <p className="text-[10px] text-red-700 leading-normal">
                            Guardian AI is currently disconnected. Please verify that the device is powered on and connected to Wi-Fi to start calibration.
                          </p>
                        </div>
                      </div>
                    )}

                    <div className="flex items-center justify-between max-w-md mx-auto">
                      <div className="flex items-center gap-2">
                        <span className={`w-7 h-7 rounded-full flex items-center justify-center text-xs font-bold ${calStep >= 1 ? 'bg-primary text-white font-bold' : 'bg-surface-container text-on-surface-variant'}`}>1</span>
                        <span className={`text-xs font-semibold ${calStep === 1 ? 'text-primary' : 'text-on-surface-variant'}`}>Test Dispense</span>
                      </div>
                      <div className="flex-1 h-0.5 bg-outline-variant mx-4"></div>
                      <div className="flex items-center gap-2">
                        <span className={`w-7 h-7 rounded-full flex items-center justify-center text-xs font-bold ${calStep >= 2 ? 'bg-primary text-white font-bold' : 'bg-surface-container text-on-surface-variant'}`}>2</span>
                        <span className={`text-xs font-semibold ${calStep === 2 ? 'text-primary' : 'text-on-surface-variant'}`}>Weigh Food</span>
                      </div>
                      <div className="flex-1 h-0.5 bg-outline-variant mx-4"></div>
                      <div className="flex items-center gap-2">
                        <span className={`w-7 h-7 rounded-full flex items-center justify-center text-xs font-bold ${calStep >= 3 ? 'bg-primary text-white font-bold' : 'bg-surface-container text-on-surface-variant'}`}>3</span>
                        <span className={`text-xs font-semibold ${calStep === 3 ? 'text-primary' : 'text-on-surface-variant'}`}>Calculate Rate</span>
                      </div>
                    </div>

                    <div className="p-6 bg-surface-container-low rounded-xl border border-outline-variant/30 max-w-xl mx-auto space-y-4">
                      {calStep === 1 && (
                        <div className="space-y-4">
                          <div className="space-y-1">
                            <h5 className="font-bold text-sm text-on-surface">Step 1: 5-Second Test Dispense</h5>
                            <p className="text-xs text-on-surface-variant">Place an empty bowl under the feeder. Press "Run 5s Test Dispense" to open the feeder lid for exactly 5 seconds.</p>
                          </div>
                          <button
                            type="button"
                            disabled={isLoading || !isDeviceOnline}
                            onClick={async () => {
                              setIsLoading(true);
                              setCalStatus('Running 5-second test dispense on device (5s remaining)...');
                              try {
                                await updateSettingsMutation({ ownerId, pendingCommand: "CMD:TARE" });
                              } catch (err: any) {
                                console.error("Error sending TARE:", err);
                              }

                              let remaining = 5;
                              const timer = setInterval(() => {
                                remaining--;
                                if (remaining > 0) {
                                  setCalStatus(`Running 5-second test dispense on device (${remaining}s remaining)...`);
                                } else {
                                  clearInterval(timer);
                                  setIsLoading(false);
                                  setCalStatus('Test dispense completed! Weigh the food and click Continue.');
                                  setCalStep(2);
                                }
                              }, 1000);
                            }}
                            className="px-5 py-2.5 bg-primary hover:bg-primary-container text-white font-bold text-xs rounded-xl flex items-center gap-2 transition-all active:scale-95 disabled:opacity-50"
                          >
                            <span className="material-symbols-outlined text-sm">timer</span>
                            {isLoading ? 'Dispensing (5s)...' : 'Run 5s Test Dispense'}
                          </button>
                        </div>
                      )}

                      {calStep === 2 && (
                        <div className="space-y-4">
                          <div className="space-y-1">
                            <h5 className="font-bold text-sm text-on-surface">Step 2: Weigh Dispensed Test Food</h5>
                            <p className="text-xs text-on-surface-variant">Weigh the test food dispensed during the 5-second run on any kitchen scale. Enter the measured weight below.</p>
                          </div>
                          <div className="flex items-end gap-4 max-w-xs">
                            <div className="flex-1">
                              <label className="text-[10px] font-bold text-on-surface-variant uppercase tracking-wider block mb-1">Weight Dispensed (grams)</label>
                              <input
                                type="number"
                                min="1"
                                max="500"
                                value={refWeightInput}
                                disabled={isLoading || !isDeviceOnline}
                                onChange={(e) => setRefWeightInput(Number(e.target.value))}
                                className="w-full px-3 py-2 border border-outline-variant rounded-xl text-sm font-bold focus:outline-none focus:border-primary disabled:opacity-50"
                              />
                            </div>
                            <button
                              type="button"
                              onClick={() => {
                                setCalStep(3);
                                setCalStatus(`Target weight set to ${refWeightInput}g. Click Apply Flow Rate to finalize.`);
                              }}
                              className="px-5 py-2.5 bg-primary hover:bg-primary-container text-white font-bold text-xs rounded-xl flex items-center gap-2 transition-all active:scale-95 h-[38px]"
                            >
                              <span className="material-symbols-outlined text-sm">arrow_forward</span>
                              Continue to Step 3
                            </button>
                          </div>
                        </div>
                      )}

                      {calStep === 3 && (
                        <div className="space-y-4">
                          <div className="space-y-1">
                            <h5 className="font-bold text-sm text-on-surface">Step 3: Calculate & Save Flow Rate</h5>
                            <p className="text-xs text-on-surface-variant">Press "Apply Flow Rate" to send the measured weight of {refWeightInput}g to the Arduino. The Arduino will calculate your feeder's exact flow rate ({ (refWeightInput / 5.0).toFixed(1) } g/sec) and store it in EEPROM.</p>
                          </div>
                          <div className="flex gap-3">
                            <button
                              type="button"
                              disabled={isLoading || !isDeviceOnline}
                              onClick={async () => {
                                setIsLoading(true);
                                setCalStatus('Calculating and saving flow rate...');
                                const calculatedRate = refWeightInput / 5.0;
                                try {
                                  await updateSettingsMutation({
                                    ownerId,
                                    pendingCommand: `CMD:CALIBRATE:${refWeightInput}`,
                                    calibrationFactor: calculatedRate
                                  });
                                  setIsLoading(false);
                                  setCalStatus(`Flow rate saved successfully: ${calculatedRate.toFixed(1)} g/sec!`);
                                } catch (err: any) {
                                  setIsLoading(false);
                                  setCalStatus(`Error applying calibration: ${err.message || err}`);
                                }
                              }}
                              className="px-5 py-2.5 bg-primary hover:bg-primary-container text-white font-bold text-xs rounded-xl flex items-center gap-2 transition-all active:scale-95 disabled:opacity-50"
                            >
                              <span className="material-symbols-outlined text-sm">check_circle</span>
                              {isLoading ? 'Saving Rate...' : 'Apply Flow Rate'}
                            </button>
                          </div>
                          {convexSettings?.calibrationFactor !== undefined && (
                            <div className="p-4 bg-green-50 border border-green-200/50 rounded-xl space-y-2">
                              <p className="text-xs text-green-800 font-bold flex items-center gap-1">
                                <span className="material-symbols-outlined text-sm">check_circle</span>
                                Flow Rate Saved
                              </p>
                              <p className="text-xs text-green-700">
                                Calculated Flow Rate: <span className="font-mono font-bold text-sm text-green-900">{convexSettings.calibrationFactor.toFixed(2)} g/sec</span>
                              </p>
                            </div>
                          )}
                        </div>
                      )}

                      {calStatus && (
                        <div className="flex items-center gap-2 text-[11px] text-on-surface-variant font-medium pt-2 border-t border-outline-variant/30">
                          {isLoading && <span className="w-2.5 h-2.5 border-2 border-primary border-t-transparent rounded-full animate-spin"></span>}
                          <span>{calStatus}</span>
                        </div>
                      )}

                      <div className="flex justify-between items-center pt-2">
                        {calStep > 1 && (
                          <button
                            type="button"
                            onClick={() => {
                              setCalStep((prev) => (prev - 1) as any);
                              setCalStatus('');
                              setIsLoading(false);
                            }}
                            className="text-xs font-bold text-on-surface-variant hover:underline"
                          >
                            Back
                          </button>
                        )}
                        <div className="flex-1"></div>
                        {calStep === 2 && convexSettings?.rawScaleValue !== undefined && !isLoading && (
                          <button
                            type="button"
                            disabled={!isDeviceOnline}
                            onClick={() => {
                              setCalStep(3);
                              setCalStatus('');
                            }}
                            className="px-4 py-2 bg-primary text-white font-bold text-xs rounded-xl disabled:opacity-50"
                          >
                            Continue
                          </button>
                        )}
                        {(calStep === 3 || convexSettings?.calibrationFactor !== undefined) && (
                          <button
                            type="button"
                            onClick={async () => {
                              setCalStep(1);
                              setCalStatus('');
                              setIsLoading(false);
                              await updateSettingsMutation({
                                ownerId,
                                rawScaleValue: undefined,
                                calibrationFactor: undefined,
                                pendingCommand: undefined
                              });
                            }}
                            className="text-xs font-bold text-primary hover:underline"
                          >
                            Start Over
                          </button>
                        )}
                      </div>
                    </div>
                  </div>
                  )}
                </div>
              </div>
            )}

            {activeTab === 'cooldown' && (
              <div className="space-y-6">
                <div className="flex items-start justify-between">
                  <div>
                    <h3 className="font-bold text-lg text-on-surface">Feeding Interval (Lockout)</h3>
                    <p className="text-xs text-on-surface-variant mt-1">Set a minimum delay in minutes between subsequent feedings to prevent overfeeding.</p>
                  </div>
                  <span className="px-3 py-1 bg-primary/10 text-primary text-[9px] font-bold rounded-full uppercase tracking-wider">Customizable</span>
                </div>
                <div className="p-6 bg-surface-bright rounded-2xl border border-outline-variant/30 max-w-xl">
                  <label className="text-sm font-bold block mb-4">Cooldown Duration (Minutes)</label>
                  <div className="flex items-center gap-4">
                    <input
                      type="range"
                      min="1"
                      max="720"
                      value={tempCooldown}
                      onChange={(e) => {
                        setCooldownDirty(true);
                        setTempCooldown(Number(e.target.value));
                      }}
                      className="flex-1 h-2 bg-surface-container rounded-lg appearance-none cursor-pointer accent-primary"
                    />
                    <input
                      type="number"
                      min="1"
                      max="720"
                      value={tempCooldown}
                      onChange={(e) => {
                        setCooldownDirty(true);
                        setTempCooldown(Number(e.target.value));
                      }}
                      onBlur={() => setTempCooldown(value => Math.max(1, Math.min(720, Math.round(value || 1))))}
                      className="w-20 px-3 py-1.5 border border-outline-variant rounded-xl text-center text-sm font-bold focus:outline-none focus:border-primary"
                    />
                  </div>
                  <div className="flex justify-between mt-3 text-xs text-on-surface-variant">
                    <span>1 min</span>
                    <span className="text-primary font-bold">
                      {tempCooldown >= 60
                        ? `${Math.floor(tempCooldown / 60)}h ${tempCooldown % 60}m`
                        : `${tempCooldown} minutes`
                      }
                    </span>
                    <span>12 hours (720m)</span>
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'temp' && (
              <div className="space-y-6">
                <div>
                  <h3 className="font-bold text-lg text-on-surface">Maximum Temperature Safety</h3>
                  <p className="text-xs text-on-surface-variant mt-1">
                    The feeder will auto-shutdown or lock dispensing if environmental temperature levels exceed this threshold.
                  </p>
                </div>
                <div className="p-8 rounded-3xl bg-gradient-to-br from-surface-bright to-surface-container-low flex flex-col items-center border border-outline-variant/30">
                  <div className="text-5xl font-bold text-primary mb-2">{tempMaxTemp}°C</div>
                  <div className="text-xs font-bold text-on-surface-variant mb-6 uppercase tracking-wider">Safety Threshold</div>
                  <div className="w-full max-w-sm">
                    <input
                      type="range"
                      min="30"
                      max="60"
                      value={tempMaxTemp}
                      onChange={(e) => setTempMaxTemp(Number(e.target.value))}
                      className="w-full h-2 bg-surface-container rounded-lg appearance-none cursor-pointer accent-primary"
                    />
                    <div className="flex justify-between text-[10px] text-on-surface-variant mt-2">
                      <span>30°C</span>
                      <span>60°C</span>
                    </div>
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'refill' && (
              <div className="space-y-6">
                <div>
                  <h3 className="font-bold text-lg text-on-surface">Refill &amp; Container Configuration</h3>
                  <p className="text-xs text-on-surface-variant mt-1">
                    Configure the initial weight of the food container when refilled. The application calculates food remaining mathematically.
                  </p>
                </div>
                <form onSubmit={handleRefillSubmit} className="space-y-4 max-w-md">
                  <div className="space-y-1">
                    <label className="text-xs font-semibold text-on-surface-variant">Container Refill Capacity (Grams)</label>
                    <input
                      type="number"
                      value={refillWeight}
                      onChange={(e) => setRefillWeight(Number(e.target.value))}
                      className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none"
                      min="100"
                      max="10000"
                    />
                  </div>
                  <button
                    type="submit"
                    className="w-full py-3 bg-primary text-on-primary font-bold text-xs rounded-xl hover:shadow-lg transition-all active:scale-95"
                  >
                    Confirm Refill / Reset Dispensed Food
                  </button>
                </form>
                <div className="p-4 rounded-xl bg-surface text-xs text-on-surface-variant border border-outline-variant/30 leading-relaxed">
                  <b>Refill logic:</b> Dispensing tracks grams used. Clicking Refill resets your total dispensed history for remaining computations.
                </div>
              </div>
            )}

            {activeTab === 'wifi' && (
              <div className="space-y-6">
                <div>
                  <h3 className="font-bold text-lg text-on-surface">Network Connectivity</h3>
                  <p className="text-xs text-on-surface-variant mt-1">Manage connection details for your Guardian AI device.</p>
                </div>
                <div className="p-6 bg-surface-bright rounded-2xl border border-outline-variant/30 flex flex-col sm:flex-row sm:items-center justify-between gap-4">
                  <div className="flex items-center gap-4">
                    <div className="w-12 h-12 bg-primary/10 text-primary rounded-xl flex items-center justify-center shrink-0">
                      <span className="material-symbols-outlined text-lg">wifi</span>
                    </div>
                    <div>
                      <p className="font-bold text-sm text-on-surface">{settings.wifiSsid}</p>
                      <p className="text-xs text-on-surface-variant">Connected • Signal: {settings.signalStrength} dBm</p>
                    </div>
                  </div>
                  <button className="px-4 py-2 border border-outline rounded-xl text-xs font-semibold hover:bg-surface-container transition-colors shrink-0">
                    Change Network
                  </button>
                </div>
                <div className="space-y-3 max-w-md">
                  <div className="space-y-1">
                    <label className="text-xs font-semibold text-on-surface-variant">Network SSID</label>
                    <input
                      type="text"
                      value={wifiSsidInput}
                      onChange={(e) => setWifiSsidInput(e.target.value)}
                      className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none"
                    />
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'info' && (
              <div className="space-y-6">
                <div>
                  <h3 className="font-bold text-lg text-on-surface">Device Information</h3>
                  <p className="text-xs text-on-surface-variant mt-1">Diagnose hardware components, firmware builds, and connectivity links.</p>
                </div>

                <div className="p-6 bg-surface rounded-2xl border border-outline-variant/30 space-y-4">
                  <h4 className="font-bold text-sm text-on-surface">ESP32-CAM Stream Configuration</h4>
                  <div className="space-y-2">
                    <label className="text-xs font-semibold text-on-surface-variant block">Camera MJPEG Stream URL</label>
                    <input
                      type="url"
                      value={streamUrl}
                      onChange={(e) => setStreamUrl(e.target.value)}
                      placeholder="http://192.168.1.100:81/stream"
                      className="w-full px-4 py-2 bg-white border border-outline-variant rounded-xl text-sm focus:outline-none focus:border-primary"
                    />
                    <p className="text-[10px] text-on-surface-variant leading-relaxed opacity-80">
                      Supply the HTTP stream URL served by your ESP32-CAM firmware (e.g. <code>http://&lt;esp32_ip&gt;/stream</code>).
                    </p>
                  </div>
                </div>

                <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
                  <div className="p-4 bg-surface-bright rounded-xl border border-outline-variant/30">
                    <p className="text-[9px] text-on-surface-variant uppercase font-bold tracking-widest mb-1">Arduino Status</p>
                    <div className="flex items-center gap-2">
                      <div className="w-2 h-2 rounded-full bg-primary animate-pulse"></div>
                      <p className="font-bold text-xs text-on-surface">{settings.arduinoStatus} (Operational)</p>
                    </div>
                  </div>
                  <div className="p-4 bg-surface-bright rounded-xl border border-outline-variant/30">
                    <p className="text-[9px] text-on-surface-variant uppercase font-bold tracking-widest mb-1">NodeMCU Status</p>
                    <div className="flex items-center gap-2">
                      <div className="w-2 h-2 rounded-full bg-primary animate-pulse"></div>
                      <p className="font-bold text-xs text-on-surface">{settings.nodemcuStatus}</p>
                    </div>
                  </div>
                  <div className="p-4 bg-surface-bright rounded-xl border border-outline-variant/30">
                    <p className="text-[9px] text-on-surface-variant uppercase font-bold tracking-widest mb-1">ESP32-CAM Status</p>
                    <div className="flex items-center gap-2">
                      <div className="w-2 h-2 rounded-full bg-primary animate-pulse"></div>
                      <p className="font-bold text-xs text-on-surface">{settings.esp32CamStatus}</p>
                    </div>
                  </div>
                  <div className="p-4 bg-surface-bright rounded-xl border border-outline-variant/30">
                    <p className="text-[9px] text-on-surface-variant uppercase font-bold tracking-widest mb-1">Firmware Version</p>
                    <p className="font-bold text-xs text-on-surface">{settings.firmwareVersion}</p>
                  </div>
                  <div className="p-4 bg-surface-bright rounded-xl border border-outline-variant/30 sm:col-span-2">
                    <p className="text-[9px] text-on-surface-variant uppercase font-bold tracking-widest mb-1">System Uptime</p>
                    <p className="font-bold text-xs text-on-surface">{settings.uptime}</p>
                  </div>
                </div>

                <div className="flex flex-col gap-3 pt-4 border-t border-outline-variant/30">
                  <button className="w-full py-3 border border-secondary text-secondary font-bold text-xs rounded-xl hover:bg-secondary/5 transition-colors flex items-center justify-center gap-2">
                    <span className="material-symbols-outlined text-sm">restart_alt</span>
                    Restart Device
                  </button>
                  <button className="w-full py-3 bg-error-container text-error font-bold text-xs rounded-xl hover:bg-error/10 transition-colors">
                    Factory Reset Device
                  </button>
                </div>
              </div>
            )}

            {activeTab === 'notifications' && (
              <div className="space-y-6">
                <div>
                  <h3 className="font-bold text-lg text-on-surface">Notification Preferences</h3>
                  <p className="text-xs text-on-surface-variant mt-1 font-medium">Control which alerts are generated and dispatched.</p>
                </div>
                <div className="space-y-2">
                  <div className="flex items-center justify-between py-4 border-b border-outline-variant/30">
                    <span className="text-sm font-semibold text-on-surface">Feeding Successful</span>
                    <label className="relative inline-flex items-center cursor-pointer">
                      <input
                        type="checkbox"
                        checked={settings.notificationsEnabled.feedingSuccessful}
                        onChange={() => toggleNotificationSetting('feedingSuccessful')}
                        className="sr-only peer"
                      />
                      <div className="w-11 h-6 bg-surface-container rounded-full peer peer-checked:after:translate-x-full peer-checked:bg-primary after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:rounded-full after:h-5 after:w-5 after:transition-all"></div>
                    </label>
                  </div>
                  <div className="flex items-center justify-between py-4 border-b border-outline-variant/30">
                    <span className="text-sm font-semibold text-on-surface">Low Food Warning</span>
                    <label className="relative inline-flex items-center cursor-pointer">
                      <input
                        type="checkbox"
                        checked={settings.notificationsEnabled.lowFoodWarning}
                        onChange={() => toggleNotificationSetting('lowFoodWarning')}
                        className="sr-only peer"
                      />
                      <div className="w-11 h-6 bg-surface-container rounded-full peer peer-checked:after:translate-x-full peer-checked:bg-primary after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:rounded-full after:h-5 after:w-5 after:transition-all"></div>
                    </label>
                  </div>
                  <div className="flex items-center justify-between py-4">
                    <span className="text-sm font-semibold text-on-surface">Critical Overheat Alerts</span>
                    <label className="relative inline-flex items-center cursor-pointer">
                      <input
                        type="checkbox"
                        checked={settings.notificationsEnabled.criticalOverheat}
                        onChange={() => toggleNotificationSetting('criticalOverheat')}
                        className="sr-only peer"
                      />
                      <div className="w-11 h-6 bg-surface-container rounded-full peer peer-checked:after:translate-x-full peer-checked:bg-primary after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:rounded-full after:h-5 after:w-5 after:transition-all"></div>
                    </label>
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'ai' && (
              <div className="space-y-6">
                <div>
                  <h3 className="font-bold text-lg text-on-surface">AI Model & Integrations</h3>
                  <p className="text-xs text-on-surface-variant mt-1 font-medium">
                    Configure the optional AI services used by the recognition system.
                  </p>
                </div>
                <div className="p-6 bg-surface-bright rounded-2xl border border-outline-variant/30 space-y-5">
                  <div className="flex items-center gap-3">
                    <span className="material-symbols-outlined text-primary">key</span>
                    <div>
                      <h4 className="font-bold text-sm text-on-surface">Chei API Vision</h4>
                      <p className="text-[11px] text-on-surface-variant">Keys are stored only in this browser.</p>
                    </div>
                  </div>
                  <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
                    <div className="space-y-2">
                      <div className="flex items-center justify-between gap-3">
                        <label htmlFor="gemini-api-key" className="text-xs font-bold text-on-surface">Gemini · Google AI Studio</label>
                        <a
                          href="https://aistudio.google.com/app/apikey"
                          target="_blank"
                          rel="noreferrer"
                          className="inline-flex items-center gap-1 text-[10px] font-bold text-primary hover:underline shrink-0"
                        >
                          Create key
                          <span className="material-symbols-outlined text-[13px]">open_in_new</span>
                        </a>
                      </div>
                      <input
                        id="gemini-api-key"
                        type="password"
                        value={geminiApiKey}
                        onChange={(event) => setGeminiApiKey(event.target.value)}
                        placeholder="Introdu cheia API Gemini"
                        autoComplete="off"
                        className="w-full px-4 py-3 rounded-xl bg-surface-container-low text-on-surface text-xs border border-outline-variant/40 focus:outline-none focus:border-primary"
                      />
                      <p className="text-[10px] text-on-surface-variant">Open Google AI Studio, generate a key, and paste it here.</p>
                    </div>
                    <div className="space-y-2">
                      <div className="flex items-center justify-between gap-3">
                        <label htmlFor="openrouter-api-key" className="text-xs font-bold text-on-surface">Qwen2-VL · OpenRouter (optional)</label>
                        <a
                          href="https://openrouter.ai/settings/keys"
                          target="_blank"
                          rel="noreferrer"
                          className="inline-flex items-center gap-1 text-[10px] font-bold text-primary hover:underline shrink-0"
                        >
                          Create key
                          <span className="material-symbols-outlined text-[13px]">open_in_new</span>
                        </a>
                      </div>
                      <input
                        id="openrouter-api-key"
                        type="password"
                        value={qwenApiKey}
                        onChange={(event) => setQwenApiKey(event.target.value)}
                        placeholder="Introdu cheia API OpenRouter"
                        autoComplete="off"
                        className="w-full px-4 py-3 rounded-xl bg-surface-container-low text-on-surface text-xs border border-outline-variant/40 focus:outline-none focus:border-primary"
                      />
                      <p className="text-[10px] text-on-surface-variant">Open the OpenRouter API Keys page, create a key, and paste it here.</p>
                    </div>
                  </div>
                </div>
                <div className="p-6 bg-surface-bright rounded-2xl border border-outline-variant/30 space-y-4">
                  <h4 className="font-bold text-sm text-on-surface flex items-center gap-2">
                    <span className="material-symbols-outlined text-purple-600">psychology</span>
                    Guardian Visual Identity Engine
                  </h4>
                  <p className="text-xs text-on-surface-variant leading-relaxed font-medium">
                    The primary CLIP engine runs on the local server. The cloud integrations above remain optional.
                  </p>
                </div>
              </div>
            )}

            <footer className="mt-12 pt-8 border-t border-outline-variant/30 flex flex-wrap items-center justify-end gap-4">
              {saveNotice && (
                <div className={`mr-auto flex items-center gap-2 px-3.5 py-2 rounded-xl text-[11px] font-bold border ${
                  saveNotice === 'success'
                    ? 'bg-emerald-50 text-emerald-700 border-emerald-200/70'
                    : 'bg-rose-50 text-rose-700 border-rose-200/70'
                }`}>
                  <span className="material-symbols-outlined text-[17px]">{saveNotice === 'success' ? 'check_circle' : 'error'}</span>
                  {saveNotice === 'success' ? 'Changes saved successfully.' : 'Changes could not be saved.'}
                </div>
              )}
              <button
                onClick={() => {
                  setTempPortion(settings.portionWeight);
                  setTempMaxTemp(settings.maxTemperature);
                  setStreamUrl(settings.cameraStreamUrl);
                  setWifiSsidInput(settings.wifiSsid);
                  setTempCooldown(Number(convexSettings?.cooldownMinutes ?? settings.cooldownMinutes));
                  setCooldownDirty(false);
                }}
                className="px-6 py-2.5 font-bold text-xs text-on-surface-variant hover:bg-surface-container-low rounded-xl transition-colors"
              >
                Cancel
              </button>
              <button
                onClick={handleSaveChanges}
                className="px-8 py-2.5 bg-primary text-white font-bold text-xs rounded-xl shadow-lg shadow-primary/20 hover:scale-[1.02] active:scale-95 transition-all"
              >
                Save Changes
              </button>
            </footer>
          </div>
        </div>
    </section>
  );
};

export default Settings;
