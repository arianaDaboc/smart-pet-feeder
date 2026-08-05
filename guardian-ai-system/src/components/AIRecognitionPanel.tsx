import React from 'react';
import type { AIInferenceResult, PetMatchState } from '../services/aiInferenceService';

interface AIRecognitionPanelProps {
  currentResult: AIInferenceResult | null;
  isAnalyzing: boolean;
  autoScanEnabled: boolean;
  confidenceThreshold: number;
  onToggleAutoScan: (enabled: boolean) => void;
  onTriggerScan: () => void;
  onChangeThreshold: (threshold: number) => void;
  isCooldown?: boolean;
  scanIntervalMs?: number;
  onChangeScanInterval?: (intervalMs: number) => void;
}

export const AIRecognitionPanel: React.FC<AIRecognitionPanelProps> = ({
  currentResult,
  isAnalyzing,
  autoScanEnabled,
  confidenceThreshold,
  onToggleAutoScan,
  onTriggerScan,
  onChangeThreshold,
  isCooldown = false,
  scanIntervalMs = 1500,
  onChangeScanInterval,
}) => {
  // Determine display state
  const getAIStatusText = () => {
    if (isCooldown) return { text: 'Cooldown Active', color: 'text-rose-500', bg: 'bg-rose-500/10' };
    if (isAnalyzing) return { text: 'Detecting...', color: 'text-amber-500', bg: 'bg-amber-500/10' };
    if (!currentResult || !currentResult.detected) return { text: 'No Animal', color: 'text-on-surface-variant', bg: 'bg-surface-variant/40' };
    if (currentResult.petMatch === 'UNKNOWN') return { text: 'Unknown', color: 'text-gray-500', bg: 'bg-gray-500/10' };
    return { text: 'Animal Detected', color: 'text-emerald-600', bg: 'bg-emerald-500/10' };
  };

  const status = getAIStatusText();

  const getPetMatchBadge = (petMatch: PetMatchState) => {
    switch (petMatch) {
      case 'AUTHORIZED':
        return (
          <span className="px-3 py-1 bg-emerald-500/15 text-emerald-700 font-bold text-xs rounded-full border border-emerald-500/30 flex items-center gap-1.5">
            <span className="w-2 h-2 rounded-full bg-emerald-500 animate-pulse"></span>
            AUTHORIZED
          </span>
        );
      case 'UNAUTHORIZED':
        return (
          <span className="px-3 py-1 bg-rose-500/15 text-rose-700 font-bold text-xs rounded-full border border-rose-500/30 flex items-center gap-1.5">
            <span className="w-2 h-2 rounded-full bg-rose-500"></span>
            UNAUTHORIZED
          </span>
        );
      case 'UNKNOWN':
      default:
        return (
          <span className="px-3 py-1 bg-slate-500/15 text-slate-700 font-bold text-xs rounded-full border border-slate-500/30 flex items-center gap-1.5">
            <span className="w-2 h-2 rounded-full bg-slate-400"></span>
            UNKNOWN
          </span>
        );
    }
  };

  return (
    <div className="bg-white rounded-3xl border border-outline-variant/30 p-6 shadow-sm space-y-6">
      {/* Header */}
      <div className="flex items-center justify-between pb-4 border-b border-outline-variant/20">
        <div className="flex items-center gap-3">
          <div className="w-10 h-10 rounded-2xl bg-primary/10 flex items-center justify-center text-primary">
            <span className="material-symbols-outlined">psychology</span>
          </div>
          <div>
            <h4 className="font-bold text-base text-on-surface">AI Animal Recognition</h4>
            <p className="text-xs text-on-surface-variant">Real-time camera frame inference</p>
          </div>
        </div>
        
        {/* Status Pill */}
        <span className={`px-3 py-1 rounded-full text-xs font-bold ${status.bg} ${status.color} ${isCooldown ? 'animate-pulse' : ''}`}>
          {status.text}
        </span>
      </div>

      {/* Primary Metrics Grid */}
      <div className="grid grid-cols-2 gap-4">
        {/* Detected Label */}
        <div className="p-4 rounded-2xl bg-surface-container-low border border-outline-variant/20">
          <p className="text-[10px] font-bold text-on-surface-variant uppercase tracking-wider mb-1">Detected Label</p>
          <p className="font-bold text-lg text-on-surface">
            {currentResult?.detected ? currentResult.label : 'None'}
          </p>
        </div>

        {/* Confidence Score */}
        <div className="p-4 rounded-2xl bg-surface-container-low border border-outline-variant/20">
          <p className="text-[10px] font-bold text-on-surface-variant uppercase tracking-wider mb-1">Confidence</p>
          <p className="font-bold text-lg text-on-surface">
            {currentResult?.detected ? `${Math.round(currentResult.confidence * 100)}%` : '0%'}
          </p>
        </div>

        {/* Pet Match Recommendation */}
        <div className="col-span-2 p-4 rounded-2xl bg-surface-container-low border border-outline-variant/20 flex justify-between items-center">
          <div>
            <p className="text-[10px] font-bold text-on-surface-variant uppercase tracking-wider mb-0.5">Pet Match</p>
            <p className="text-[11px] text-on-surface-variant leading-none">Authorization status</p>
          </div>
          {getPetMatchBadge(currentResult?.petMatch ?? 'UNKNOWN')}
        </div>
      </div>

      {/* AI Model Diagnostic Information Panel */}
      <div className="p-4 rounded-2xl bg-slate-950 text-white space-y-2">
        <div className="flex justify-between items-center text-xs">
          <span className="font-bold text-slate-400 tracking-widest text-[9px] uppercase">AI Model</span>
          <span className="px-2 py-0.5 rounded bg-emerald-500/20 text-emerald-400 font-bold text-[10px]">
            {currentResult?.modelInfo.name || 'Guardian Visual Identity'}
          </span>
        </div>
        <div className="flex justify-between items-center text-xs pt-1">
          <span className="text-slate-300 font-medium">Model Version</span>
          <span className="font-mono text-slate-200">{currentResult?.modelInfo.version || 'Local CLIP'}</span>
        </div>
        <div className="flex justify-between items-center text-xs">
          <span className="text-slate-300 font-medium">Inference Latency</span>
          <span className="font-mono text-slate-200">{currentResult?.modelInfo.inferenceTimeMs ?? 0} ms</span>
        </div>
        <div className="flex justify-between items-center text-xs pt-1 border-t border-white/10">
          <span className="text-slate-300 font-medium">Sync Window</span>
          <span className="font-mono text-emerald-400 font-bold">10s PIR & AI Sync</span>
        </div>
      </div>

      {/* Frame Refresh Rate Selector */}
      {onChangeScanInterval && (
        <div className="space-y-2 pt-1">
          <div className="flex justify-between items-center text-xs font-bold text-on-surface-variant">
            <span>Camera Refresh Speed</span>
            <span className="material-symbols-outlined text-[16px] text-emerald-500">speed</span>
          </div>
          <select
            value={scanIntervalMs}
            onChange={(e) => onChangeScanInterval(parseInt(e.target.value, 10))}
            disabled={isCooldown}
            className="w-full bg-surface-container-low border border-outline-variant/30 rounded-xl px-3 py-2.5 text-xs font-semibold text-on-surface focus:outline-none focus:border-primary transition-colors cursor-pointer disabled:opacity-50"
          >
            <option value={1500}>1.5s (Real-Time Live Capture)</option>
            <option value={3000}>3.0s (Fast Refresh)</option>
            <option value={5000}>5.0s (Standard Refresh)</option>
          </select>
        </div>
      )}

      {/* Threshold & Auto-Scan Controls */}
      <div className="space-y-4 pt-2">
        {/* Confidence Threshold Slider */}
        <div className="space-y-1">
          <div className="flex justify-between items-center text-xs font-bold text-on-surface-variant">
            <span>Confidence Threshold</span>
            <span>{Math.round(confidenceThreshold * 100)}%</span>
          </div>
          <input
            type="range"
            min="0.80"
            max="0.98"
            step="0.01"
            value={confidenceThreshold}
            onChange={(e) => onChangeThreshold(parseFloat(e.target.value))}
            disabled={isCooldown}
            className="w-full h-1.5 bg-outline-variant/30 rounded-lg appearance-none cursor-pointer accent-primary disabled:opacity-50"
          />
        </div>

        {/* Scan Action Buttons */}
        <div className="flex items-center justify-between gap-4 pt-2">
          {/* Auto-Scan Toggle */}
          <label className={`flex items-center gap-3 cursor-pointer ${isCooldown ? 'opacity-50 pointer-events-none' : ''}`}>
            <input
              type="checkbox"
              checked={autoScanEnabled}
              onChange={(e) => onToggleAutoScan(e.target.checked)}
              disabled={isCooldown}
              className="sr-only peer"
            />
            <div className="w-9 h-5 bg-outline-variant peer-focus:outline-none rounded-full peer peer-checked:after:translate-x-full peer-checked:after:border-white after:content-[''] after:absolute after:top-[2px] after:left-[2px] after:bg-white after:border-gray-300 after:border after:rounded-full after:h-4 after:w-4 after:transition-all peer-checked:bg-primary relative"></div>
            <span className="text-xs font-bold text-on-surface">Auto-Scan ({scanIntervalMs / 1000}s)</span>
          </label>

          {/* Manual Trigger */}
          <button
            onClick={onTriggerScan}
            disabled={isAnalyzing || isCooldown}
            className="px-4 py-2 bg-primary hover:bg-primary-container text-white text-xs font-bold rounded-xl transition-all shadow-md active:scale-95 disabled:opacity-50 flex items-center gap-2"
          >
            <span className="material-symbols-outlined text-sm">center_focus_weak</span>
            {isAnalyzing ? 'Scanning...' : 'Scan Frame Now'}
          </button>
        </div>
      </div>
    </div>
  );
};
