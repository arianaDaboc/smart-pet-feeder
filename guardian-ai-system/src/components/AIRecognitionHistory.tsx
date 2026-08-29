import React from 'react';
import type { PetMatchState } from '../services/aiInferenceService';

export interface AIRecognitionRecord {
  id: string;
  timestamp: number;
  speciesDetected: string;
  confidence: number;
  authorized: boolean;
  petMatch?: PetMatchState | string;
  recognizedPetName?: string;
  imageUrl: string;
}

interface AIRecognitionHistoryProps {
  records: AIRecognitionRecord[];
  onClearHistory?: () => void;
}

export const AIRecognitionHistory: React.FC<AIRecognitionHistoryProps> = ({ records, onClearHistory }) => {
  const getBadge = (petMatch?: string, authorized?: boolean) => {
    const matchState = petMatch || (authorized ? 'AUTHORIZED' : 'UNAUTHORIZED');

    switch (matchState) {
      case 'AUTHORIZED':
        return <span className="px-2.5 py-0.5 rounded-full text-[10px] font-bold bg-emerald-500/10 text-emerald-700">AUTHORIZED</span>;
      case 'UNAUTHORIZED':
        return <span className="px-2.5 py-0.5 rounded-full text-[10px] font-bold bg-rose-500/10 text-rose-700">UNAUTHORIZED</span>;
      default:
        return <span className="px-2.5 py-0.5 rounded-full text-[10px] font-bold bg-slate-500/10 text-slate-700">UNKNOWN</span>;
    }
  };

  return (
    <div className="bg-white rounded-3xl border border-outline-variant/30 p-6 shadow-sm space-y-4">
      <div className="flex items-center justify-between pb-3 border-b border-outline-variant/20">
        <div>
          <h4 className="font-bold text-base text-on-surface">AI Recognition History</h4>
          <p className="text-xs text-on-surface-variant">Log of camera frame object recognitions</p>
        </div>
        {onClearHistory && records.length > 0 && (
          <button
            onClick={onClearHistory}
            className="text-xs font-bold text-error hover:underline"
          >
            Clear History
          </button>
        )}
      </div>

      {records.length === 0 ? (
        <div className="py-8 text-center text-on-surface-variant text-xs space-y-2">
          <span className="material-symbols-outlined text-3xl opacity-40">history</span>
          <p className="font-medium">No recognition logs recorded yet.</p>
        </div>
      ) : (
        <div className="space-y-3 max-h-[350px] overflow-y-auto pr-1">
          {records.map((rec) => (
            <div
              key={rec.id}
              className="flex items-center justify-between p-3 rounded-2xl bg-surface-container-low hover:bg-surface-container transition-colors border border-outline-variant/15"
            >
              <div className="flex items-center gap-3">
                <div className="w-10 h-10 rounded-xl overflow-hidden bg-black/10 flex-shrink-0">
                  {rec.imageUrl ? (
                    <img src={rec.imageUrl} alt={rec.speciesDetected} className="w-full h-full object-cover" />
                  ) : (
                    <div className="w-full h-full flex items-center justify-center text-on-surface-variant">
                      <span className="material-symbols-outlined text-sm">pets</span>
                    </div>
                  )}
                </div>
                <div>
                  <p className="font-bold text-sm text-on-surface">
                    {rec.speciesDetected} {rec.recognizedPetName ? `(${rec.recognizedPetName})` : ''}
                  </p>
                  <p className="text-[11px] text-on-surface-variant">
                    {new Date(rec.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit' })} • Confidence: {Math.round(rec.confidence < 1 ? rec.confidence * 100 : rec.confidence)}%
                  </p>
                </div>
              </div>
              <div>
                {getBadge(rec.petMatch, rec.authorized)}
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};
