import { create } from 'zustand';
import type { FeederSettings, AISettings } from '../types';
import { convex, convexHttp } from '../lib/convex';
import { api } from '../../convex/_generated/api';

interface FeederState {
  settings: FeederSettings;
  aiSettings: AISettings;
  totalDispensed: number;
  isDispensing: boolean;
  updateSettings: (newSettings: Partial<FeederSettings>) => void;
  updateAISettings: (newSettings: Partial<AISettings>) => void;
  dispenseFood: (ownerId: string, amount?: number, aiAuthorized?: boolean) => Promise<boolean>;
  syncFromConvex: (convexSettings: any) => void;
}

export const useFeederStore = create<FeederState>((set) => ({
  settings: {
    portionWeight: 45,
    cooldownMinutes: 30,
    maxTemperature: 42,
    initialFoodAdded: 2000,
    wifiSsid: 'Home-Network-2G',
    wifiSignal: 'Excellent',
    signalStrength: -62,
    systemOnline: true,
    lastSyncTime: '3 seconds ago',
    uptime: '14 Days, 3 Hours, 22 Minutes',
    firmwareVersion: 'v2.4.1 (Stable)',
    arduinoStatus: 'Active',
    nodemcuStatus: 'Connected',
    esp32CamStatus: 'Streaming Ready',
    cameraStreamUrl: '',
    notificationsEnabled: {
      feedingSuccessful: true,
      lowFoodWarning: true,
      criticalOverheat: true
    }
  },
  aiSettings: {
    model1Enabled: true,
    model2Enabled: true,
    notificationsEnabled: true
  },
  totalDispensed: 0,
  isDispensing: false,

  updateSettings: (newSettings) => set((state) => ({
    settings: { ...state.settings, ...newSettings }
  })),

  updateAISettings: (newAISettings) => set((state) => ({
    aiSettings: { ...state.aiSettings, ...newAISettings }
  })),

  dispenseFood: async (ownerId, _amount, aiAuthorized = false) => {
    set({ isDispensing: true });

    try {
      const client = convexHttp ?? convex;
      const amountToDispense = typeof _amount === 'number' && _amount > 0 ? _amount : 45;
      let authorizationDelivered = false;

      const serialPort = (window as any).__activeSerialPort;
      if (serialPort && serialPort.writable) {
        try {
          const writer = serialPort.writable.getWriter();
          const encoder = new TextEncoder();
          await writer.write(encoder.encode(aiAuthorized ? "CMD:WAIT_MOTION\n" : "CMD:MANUAL_FEED\n"));
          writer.releaseLock();
          authorizationDelivered = true;
          console.log("AI authorization sent; waiting up to 10s for PIR confirmation.");
        } catch (serialErr) {
          console.warn("USB Serial feed write failed:", serialErr);
        }
      }

      const nodeMcuBaseUrl = (localStorage.getItem('guardian_nodemcu_url') || 'http://192.168.100.50').replace(/\/$/, '');
      if (!authorizationDelivered) {
        try {
          const response = await fetch(`${nodeMcuBaseUrl}/${aiAuthorized ? 'authorize' : 'manual-feed'}`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ autorizat: aiAuthorized, authorized: aiAuthorized, amount: amountToDispense })
          });
          if (response.status === 409) {
            console.warn('Manual dispense blocked: feeder is in cooldown.');
            set({ isDispensing: false });
            return false;
          }
          authorizationDelivered = response.ok;
        } catch (error) {
          console.warn("Direct NodeMCU authorization unavailable; using Convex fallback.", error);
        }
      }

      try {
        const localHistory = JSON.parse(localStorage.getItem('guardian_local_feed_history') || '[]');
        const newEvent = {
          _id: 'local_feed_' + Date.now(),
          timestamp: Date.now(),
          amountDispensed: amountToDispense,
          completed: !aiAuthorized,
          feedingMethod: aiAuthorized ? 'AI authorized; PIR confirmation pending' : 'Manual App Dispense'
        };
        localStorage.setItem('guardian_local_feed_history', JSON.stringify([newEvent, ...localHistory]));
        window.dispatchEvent(new Event('storage'));
      } catch (e) {}

      const targetOwnerIds = Array.from(new Set([ownerId, "user_test", "demo_owner_id"])).filter(Boolean);
      for (const targetId of targetOwnerIds) {
        if (client) {
          client.mutation(api.deviceSettings.update, {
            ownerId: targetId,
            foodPortion: amountToDispense,
            pendingFeedRequest: false,
            pendingCommand: authorizationDelivered ? "" : (aiAuthorized ? "CMD:WAIT_MOTION" : "CMD:MANUAL_FEED"),
            deviceStatus: "VERIFYING",
          }).catch(() => {});
        }

        fetch("https://amiable-starling-324.convex.site/api/telemetry", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({
            ownerId: targetId,
            pendingFeedRequest: false,
            pendingCommand: authorizationDelivered ? "" : (aiAuthorized ? "CMD:WAIT_MOTION" : "CMD:MANUAL_FEED"),
            deviceStatus: "VERIFYING",
            online: true,
          })
        }).catch(() => {});
      }

      setTimeout(() => {
        set({ isDispensing: false });
      }, 3000);

      return true;
    } catch (err) {
      console.error('Failed to request food dispensing:', err);
      set({ isDispensing: false });
      return false;
    }
  },

  syncFromConvex: (convexSettings) => set((state) => {
    if (!convexSettings) return {};

    const initial = convexSettings.initialFoodAmount ?? 2000;
    const remaining = convexSettings.estimatedFoodRemaining ?? 2000;
    const totalDispensed = Math.max(0, initial - remaining);

    const isPending = !!convexSettings.pendingFeedRequest;
    const isBusyStatus = convexSettings.deviceStatus === "FEEDING" || convexSettings.deviceStatus === "VERIFYING";

    const newIsDispensing = isPending || isBusyStatus;

    return {
      totalDispensed,
      isDispensing: newIsDispensing,
      settings: {
        ...state.settings,
        portionWeight: convexSettings.foodPortion,
        cooldownMinutes: convexSettings.cooldownMinutes ?? 30,
        maxTemperature: convexSettings.maximumTemperature,
        initialFoodAdded: initial,
        cameraStreamUrl: convexSettings.cameraStreamUrl,
        wifiSsid: convexSettings.wifiSSID,
        notificationsEnabled: {
          feedingSuccessful: convexSettings.notificationsEnabled?.feedingSuccessful ?? true,
          lowFoodWarning: convexSettings.notificationsEnabled?.lowFoodWarning ?? true,
          criticalOverheat: convexSettings.notificationsEnabled?.criticalOverheat ?? true
        }
      }
    };
  })
}));
