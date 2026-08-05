import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useUser } from '@clerk/clerk-react';
import { api } from '../../convex/_generated/api';
import { useFeederStore } from '../store/useFeederStore';
import { smartAIEngine } from '../services/aiInferenceService';
import type { AIInferenceResult, ReferencePhoto } from '../services/aiInferenceService';
import { AIRecognitionPanel } from '../components/AIRecognitionPanel';
import { AIRecognitionHistory } from '../components/AIRecognitionHistory';
import type { AIRecognitionRecord } from '../components/AIRecognitionHistory';
import { useSafeQuery, useSafeMutation } from '../lib/useSafeConvex';

export const LiveCamera: React.FC = () => {
  const { user } = useUser();
  const ownerId = user?.id || 'demo_owner_id';

  const { isDispensing, dispenseFood, syncFromConvex } = useFeederStore();

  const rawSettings = useSafeQuery(api.deviceSettings.get, { ownerId }, null);
  const rawPets = useSafeQuery(api.pets.list, { ownerId }, []);
  const rawDetections = useSafeQuery(api.aiRecognition.list, { ownerId }, []);

  const convexSettings: any = rawSettings || { deviceStatus: "STANDBY", foodPortion: 45, cameraStreamUrl: "http://192.168.100.49:81/stream", cooldownMinutes: 1 };

  // FIX: keep isDispensing synced to the REAL device status from Convex,
  // instead of relying only on a blind local timeout.
  useEffect(() => {
    if (rawSettings) syncFromConvex(rawSettings);
  }, [rawSettings, syncFromConvex]);

  // Combine Convex pets with local storage fallback pets so they persist across reloads
  const localSavedPets = (() => {
    try {
      const saved = localStorage.getItem('guardian_local_pets');
      if (!saved || JSON.parse(saved).length === 0) {
        const defaultPets = [{ _id: 'pet_default_1', name: 'Max', species: 'Cat', breed: 'European', age: '2 years' }];
        localStorage.setItem('guardian_local_pets', JSON.stringify(defaultPets));
        return defaultPets;
      }
      return JSON.parse(saved);
    } catch {
      return [];
    }
  })();

  const registeredPets = [...(rawPets || []), ...localSavedPets].filter((pet, index, self) => 
    index === self.findIndex((p) => (p._id || p.id || p.name) === (pet._id || pet.id || pet.name))
  );
  const hasNoPetsRegistered = registeredPets.length === 0;

  const detections = rawDetections || [];

  const addAIRecognitionMutation = useSafeMutation(api.aiRecognition.add);
  const clearAIRecognitionMutation = useSafeMutation(api.aiRecognition.clear);
  const addNotificationMutation = useSafeMutation(api.notifications.add);
  const updateDeviceSettingsMutation = useSafeMutation(api.deviceSettings.update);

  // System activity logs state
  interface SystemLog {
    id: string;
    time: string;
    text: string;
    type: 'info' | 'success' | 'warning';
  }
  const [logs, setLogs] = useState<SystemLog[]>([]);
  const [snapshotTimestamp, setSnapshotTimestamp] = useState<string>('Never');

  const addLog = useCallback((text: string, type: 'info' | 'success' | 'warning' = 'info') => {
    const time = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
    setLogs((prev) => [
      ...prev.slice(-49),
      { id: Math.random().toString(), time, text, type },
    ]);
  }, []);

  const logContainerRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    if (logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  }, [logs]);

  useEffect(() => {
    if (hasNoPetsRegistered) {
      addLog("WARNING: No pets are registered. All detections will remain UNAUTHORIZED until a pet is added.", "warning");
    }
  }, [hasNoPetsRegistered, addLog]);

  // AI State
  const [aiResult, setAiResult] = useState<AIInferenceResult | null>(null);
  const [isAnalyzing, setIsAnalyzing] = useState(false);
  const [autoScanEnabled, setAutoScanEnabled] = useState<boolean>(() => {
    const saved = localStorage.getItem('guardian_autoScanEnabled');
    return saved !== null ? saved === 'true' : true;
  });
  const [confidenceThreshold, setConfidenceThreshold] = useState<number>(() => {
    const saved = localStorage.getItem('guardian_confidenceThreshold');
    const parsed = saved !== null ? parseFloat(saved) : 0.88;
    return Number.isFinite(parsed) ? Math.min(0.98, Math.max(0.80, parsed)) : 0.88;
  });
  const [aiMode] = useState<string>('real');

  // Teachable Machine Model URL
  const [teachableModelUrl] = useState<string>(() => {
    return localStorage.getItem('guardian_teachable_model_url') || localStorage.getItem('guardian_teachableModelUrl') || '';
  });
  useEffect(() => {
    localStorage.setItem('guardian_teachable_model_url', teachableModelUrl);
    localStorage.setItem('guardian_teachableModelUrl', teachableModelUrl);
  }, [teachableModelUrl]);

  // Gemini & Qwen Vision API keys + reference photos
  const [geminiApiKey] = useState<string>(() => {
    return localStorage.getItem('guardian_feeder_gemini_api_key') || localStorage.getItem('guardian_gemini_api_key') || '';
  });
  useEffect(() => {
    localStorage.setItem('guardian_feeder_gemini_api_key', geminiApiKey);
    localStorage.setItem('guardian_gemini_api_key', geminiApiKey);
  }, [geminiApiKey]);

  const [qwenApiKey] = useState<string>(() => {
    return localStorage.getItem('guardian_qwen_api_key') || localStorage.getItem('guardian_openrouter_api_key') || '';
  });
  useEffect(() => {
    localStorage.setItem('guardian_qwen_api_key', qwenApiKey);
    localStorage.setItem('guardian_openrouter_api_key', qwenApiKey);
  }, [qwenApiKey]);

  const [referencePhotosMap] = useState<Record<string, string>>(() => {
    try {
      const saved = localStorage.getItem('guardian_referencePhotos');
      return saved ? JSON.parse(saved) : {};
    } catch {
      return {};
    }
  });
  useEffect(() => {
    localStorage.setItem('guardian_referencePhotos', JSON.stringify(referencePhotosMap));
  }, [referencePhotosMap]);

  const [streamError, setStreamError] = useState(false);
  const [useDemoVideo] = useState(false);
  const [snapshotMode] = useState(true);

  const customCameraUrl = localStorage.getItem('guardian_custom_camera_url') || "";

  // Stream URL calculation
  const rawStreamUrl = customCameraUrl || convexSettings?.cameraStreamUrl || "";
  const isUnsplashFake = rawStreamUrl.includes('unsplash.com');
  const baseStreamUrl = (rawStreamUrl && !isUnsplashFake) ? rawStreamUrl : "http://192.168.100.49:81/stream";
  const baseCaptureUrl = baseStreamUrl.replace('/stream', '/capture');
  const [captureUrl, setCaptureUrl] = useState(baseCaptureUrl);

  useEffect(() => {
    localStorage.setItem('guardian_autoScanEnabled', String(autoScanEnabled));
  }, [autoScanEnabled]);

  useEffect(() => {
    localStorage.setItem('guardian_confidenceThreshold', String(confidenceThreshold));
  }, [confidenceThreshold]);

  useEffect(() => {
    localStorage.setItem('guardian_aiMode', aiMode);
  }, [aiMode]);

  useEffect(() => {
    if (!ownerId) return;
    if (aiMode !== 'real' && aiMode !== 'reference-test') return;
    const cmd = streamError ? "CMD:CAM_OFFLINE" : "CMD:CAM_ONLINE";
    updateDeviceSettingsMutation({ ownerId, pendingCommand: cmd });
  }, [streamError, aiMode, ownerId]);

  // Refs
  const triggerScanRef = useRef<any>(null);
  const nextSnapshotTimeoutRef = useRef<any>(null);
  const consecutiveAuthorizedCountRef = useRef(0);
  const lastCandidatePetIdRef = useRef<string>("");
  const verificationRetryCountRef = useRef(0);
  const hasProcessedMotionRef = useRef(false);
  const lastMotionTimestampRef = useRef<number>(0);
  const lastAnimalDetectionTimestampRef = useRef<number>(0);
  const lastAuthorizedPetDetectedTimestampRef = useRef(0);
  const lastAuthorizedPetNameRef = useRef<string>("");
  const lastNoDetectionLogRef = useRef<number>(0);
  const [scanIntervalMs, setScanIntervalMs] = useState<number>(1500);
  const wasFeederBusyRef = useRef(false);
  const feederStatus = convexSettings?.deviceStatus || "STANDBY";
  const isCooldown = feederStatus === "COOLDOWN";
  const isFeeding = feederStatus === "FEEDING";
  const isVerifying = feederStatus === "VERIFYING";
  const isFeederBusy = isCooldown || isFeeding || isVerifying;

  const [urgentAlertActive, setUrgentAlertActive] = useState(false);
  const unrecognizedStartTimeRef = useRef<number | null>(null);
  const urgentAlertSentRef = useRef(false);

  useEffect(() => {
    addLog("Guardian AI: monitoring active (STANDBY). The camera is scanning in real time...", "info");
  }, [addLog]);

  useEffect(() => {
    if (!ownerId) return;
    const cooldownMin = convexSettings?.cooldownMinutes || 1;
    const urgentThresholdMs = 2 * cooldownMin * 60 * 1000;

    const interval = setInterval(async () => {
      if (!unrecognizedStartTimeRef.current) {
        unrecognizedStartTimeRef.current = Date.now();
      } else {
        const elapsed = Date.now() - unrecognizedStartTimeRef.current;
        if (elapsed >= urgentThresholdMs && !urgentAlertSentRef.current) {
          urgentAlertSentRef.current = true;
          setUrgentAlertActive(true);
          addLog(`URGENT ALERT: Pet missed feeding window (${Math.round(elapsed / 60000)} minutes elapsed without confirmed recognition).`, 'warning');
          try {
            await addNotificationMutation({
              ownerId,
              title: "URGENT: Pet Missed Feeding Window!",
              message: `URGENT ALERT: Feeder has been unable to confirm pet recognition for ${Math.round(elapsed / 60000)} minutes (2 full cooldown periods). Please inspect camera feed or feed manually!`,
              type: "Warning",
            });
          } catch (e) {
            console.error("Failed to send urgent notification:", e);
          }
        }
      }
    }, 5000);

    return () => clearInterval(interval);
  }, [ownerId, convexSettings?.cooldownMinutes]);

  useEffect(() => {
    setCaptureUrl(baseCaptureUrl);
  }, [baseCaptureUrl]);

  useEffect(() => {
    if ((aiMode !== 'real' && aiMode !== 'reference-test') || useDemoVideo) {
      setStreamError(false);
      return;
    }
    setStreamError(false);
    const timer = setTimeout(() => {
      const img = document.querySelector<HTMLImageElement>('img[alt="ESP32-CAM Live Stream"]');
      if (!img || img.naturalWidth === 0) {
        console.warn("Camera stream connection timed out after 10 seconds");
        setStreamError(true);
      }
    }, 10000);
    return () => clearTimeout(timer);
  }, [baseStreamUrl, aiMode, useDemoVideo]);

  useEffect(() => {
    if (!streamError || (aiMode !== 'real' && aiMode !== 'reference-test') || useDemoVideo) return;
    const interval = setInterval(() => {
      addLog("Attempting automatic camera stream reconnection...", "info");
      setCaptureUrl(`${baseCaptureUrl}?t=${Date.now()}`);
    }, 5000);
    return () => clearInterval(interval);
  }, [streamError, baseCaptureUrl, aiMode, useDemoVideo]);

  const sendHTTPResultToESP32 = async (isAuthorized: boolean) => {
    const streamUrl = baseStreamUrl;
    const baseUrl = streamUrl.replace(/\/stream.*$/, '').replace(/\/capture.*$/, '');
    const resultEndpoint = `${baseUrl}/result`;
    const payload = JSON.stringify({ autorizat: isAuthorized, authorized: isAuthorized });

    addLog(`Visual HTTP POST notification sent to ESP32-CAM (${resultEndpoint})...`, "info");

    try {
      fetch(resultEndpoint, {
        method: "POST",
        headers: { "Content-Type": "text/plain" },
        body: payload
      }).catch(() => {});
    } catch (e) {}

    try {
      fetch(resultEndpoint, {
        method: "POST",
        mode: "no-cors",
        body: payload
      }).catch(() => {});
    } catch (e) {}

    try {
      fetch(resultEndpoint, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: payload
      }).catch(() => {});
    } catch (e) {}

    if (baseUrl.includes(':81')) {
      const port80Endpoint = baseUrl.replace(':81', '') + '/result';
      fetch(port80Endpoint, {
        method: "POST",
        headers: { "Content-Type": "text/plain" },
        body: payload
      }).catch(() => {});
    }

    addLog("Visual notification received by ESP32-CAM.", "info");
  };

  const fetchUntaintedImageElement = async (
    targetUrl: string,
    existingImgEl?: HTMLImageElement | HTMLVideoElement
  ): Promise<{ unTaintedImg?: HTMLImageElement | HTMLVideoElement; base64?: string; dataUrl?: string }> => {
    if (existingImgEl instanceof HTMLVideoElement) {
      return { unTaintedImg: existingImgEl };
    }
    if (existingImgEl instanceof HTMLImageElement && existingImgEl.src && existingImgEl.src.startsWith('data:image/')) {
      const dataUrl = existingImgEl.src;
      const base64 = dataUrl.split(',')[1];
      return { unTaintedImg: existingImgEl, base64, dataUrl };
    }

    try {
      const fetchUrl = targetUrl || (existingImgEl instanceof HTMLImageElement ? existingImgEl.src : '');
      if (!fetchUrl) return { unTaintedImg: existingImgEl };

      const res = await fetch(fetchUrl, { cache: 'no-cache' });
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      const blob = await res.blob();

      return new Promise((resolve) => {
        const reader = new FileReader();
        reader.onloadend = () => {
          const dataUrl = reader.result as string;
          const base64 = dataUrl.split(',')[1];
          const untaintedImg = new Image();
          untaintedImg.crossOrigin = 'anonymous';
          untaintedImg.onload = () => resolve({ unTaintedImg: untaintedImg, base64, dataUrl });
          untaintedImg.onerror = () => resolve({ unTaintedImg: existingImgEl, base64, dataUrl });
          untaintedImg.src = dataUrl;
        };
        reader.onerror = () => resolve({ unTaintedImg: existingImgEl });
        reader.readAsDataURL(blob);
      });
    } catch (err) {
      console.warn("Direct blob fetch for untainted CORS capture failed:", err);
      return { unTaintedImg: existingImgEl };
    }
  };

  const handleTriggerScan = async (overrideMode?: string) => {
    if (isAnalyzing || !ownerId) return;
    setIsAnalyzing(true);

    try {
      const petsList = registeredPets.map((p: any) => ({
        id: p._id || p.id,
        name: p.name,
        species: p.species || 'Cat'
      }));

      const streamImgEl = document.querySelector<HTMLImageElement | HTMLVideoElement>(
        useDemoVideo
          ? 'video#demo-video-stream'
          : 'img#esp32-cam-stream, img[alt="ESP32-CAM Live Stream"]'
      ) || undefined;

      const targetFetchUrl = snapshotMode ? captureUrl : baseCaptureUrl;
      const { unTaintedImg, base64: untaintedBase64, dataUrl: untaintedDataUrl } = await fetchUntaintedImageElement(targetFetchUrl, streamImgEl);
      const finalImgEl = unTaintedImg || streamImgEl;

      const multiRefMap = JSON.parse(localStorage.getItem('guardian_referencePhotos_multi') || '{}');
      const referencePhotos: ReferencePhoto[] = [];

      const remoteImageToBase64 = async (url: string): Promise<string | null> => {
        try {
          if (url.startsWith('data:image/')) return url.split(',')[1] || null;
          const response = await fetch(url, { cache: 'force-cache' });
          if (!response.ok) return null;
          const blob = await response.blob();
          return await new Promise<string | null>((resolve) => {
            const reader = new FileReader();
            reader.onloadend = () => resolve(String(reader.result || '').split(',')[1] || null);
            reader.onerror = () => resolve(null);
            reader.readAsDataURL(blob);
          });
        } catch {
          return null;
        }
      };

      for (const p of petsList) {
        if (multiRefMap[p.id] && Array.isArray(multiRefMap[p.id]) && multiRefMap[p.id].length > 0) {
          for (const b64 of multiRefMap[p.id]) {
            referencePhotos.push({ petId: p.id, name: p.name, imageBase64: b64 });
          }
        } else if (referencePhotosMap[p.id]) {
          referencePhotos.push({ petId: p.id, name: p.name, imageBase64: referencePhotosMap[p.id] });
        } else {
          // Restore reference images from Convex when this browser has no local cache.
          const storedPet: any = registeredPets.find((pet: any) => String(pet._id || pet.id) === String(p.id));
          const remoteImages = [storedPet?.profileImage, ...(storedPet?.trainingImages || [])]
            .filter((value: unknown): value is string => typeof value === 'string' && value.length > 0)
            .slice(0, 6);
          const restored: string[] = [];
          for (const imageUrl of remoteImages) {
            const base64 = await remoteImageToBase64(imageUrl);
            if (base64) {
              restored.push(base64);
              referencePhotos.push({ petId: p.id, name: p.name, imageBase64: base64 });
            }
          }
          if (restored.length > 0) {
            multiRefMap[p.id] = restored;
            localStorage.setItem('guardian_referencePhotos_multi', JSON.stringify(multiRefMap));
          }
        }
      }

      // Synchronize each saved reference with the persistent local CLIP database once.
      const localAiUrl = (localStorage.getItem('guardian_local_ai_url') || 'http://127.0.0.1:3000').replace(/\/$/, '');
      const enrollmentCache: Record<string, boolean> = JSON.parse(localStorage.getItem('guardian_clip_enrollment_cache') || '{}');
      for (const reference of referencePhotos) {
        const fingerprint = `clip-v3-safe-threshold:${reference.petId}:${reference.imageBase64.length}:${reference.imageBase64.slice(-64)}`;
        if (enrollmentCache[fingerprint]) continue;
        try {
          const imageResponse = await fetch(`data:image/jpeg;base64,${reference.imageBase64}`);
          const form = new FormData();
          form.append('petId', String(reference.petId));
          form.append('petName', reference.name);
          form.append('image', await imageResponse.blob(), `${reference.name}.jpg`);
          const response = await fetch(`${localAiUrl}/api/enroll`, { method: 'POST', body: form });
          if (response.ok) {
            enrollmentCache[fingerprint] = true;
            localStorage.setItem('guardian_clip_enrollment_cache', JSON.stringify(enrollmentCache));
          }
        } catch (error) {
          console.warn('Local CLIP enrollment unavailable; cloud identity fallback will be used.', error);
          break;
        }
      }

      const result = await smartAIEngine.recognize({
        imageElement: finalImgEl,
        imageBase64: untaintedBase64,
        registeredPets: petsList,
        confidenceThreshold,
        aiMode: overrideMode || aiMode,
        teachableModelUrl: teachableModelUrl || undefined,
        referencePhotos: referencePhotos.length > 0 ? referencePhotos : undefined,
        qwenApiKey: qwenApiKey || undefined,
      });

      console.log("[AI Scan]", {
        detected: result.detected,
        label: result.label,
        confidence: result.confidence,
        petMatch: result.petMatch,
        diagnostics: result.diagnostics,
      });

      setAiResult(result);
      localStorage.setItem('guardian_last_ai_detection', JSON.stringify({
        timestamp: Date.now(),
        recognizedPetName: result.recognizedPetName,
        recognizedPetId: result.recognizedPetId,
        speciesDetected: result.label,
        confidence: result.confidence,
        authorized: result.petMatch === 'AUTHORIZED',
        petMatch: result.petMatch,
        modelName: result.modelInfo?.name,
        modelVersion: result.modelInfo?.version,
        inferenceTimeMs: result.modelInfo?.inferenceTimeMs,
      }));
      window.dispatchEvent(new Event('guardian-ai-detection-update'));
      let triggeredFeed = false;

      let capturedImageUrl = untaintedDataUrl || captureUrl || baseCaptureUrl;
      if (!capturedImageUrl && finalImgEl) {
        try {
          const canvas = document.createElement('canvas');
          if (finalImgEl instanceof HTMLVideoElement) {
            canvas.width = finalImgEl.videoWidth || 320;
            canvas.height = finalImgEl.videoHeight || 240;
          } else if (finalImgEl instanceof HTMLImageElement) {
            canvas.width = finalImgEl.naturalWidth || finalImgEl.width || 320;
            canvas.height = finalImgEl.naturalHeight || finalImgEl.height || 240;
          } else {
            canvas.width = 320;
            canvas.height = 240;
          }
          const ctx = canvas.getContext('2d');
          if (ctx) {
            ctx.drawImage(finalImgEl, 0, 0, canvas.width, canvas.height);
            capturedImageUrl = canvas.toDataURL('image/jpeg', 0.75);
          }
        } catch (e) {
          console.warn("Failed to capture camera frame:", e);
        }
      }
      if (!capturedImageUrl) {
        capturedImageUrl = captureUrl || baseCaptureUrl;
      }

      if (result.detected) {
        if (result.petMatch === 'AUTHORIZED') {
          const now = Date.now();
          lastAnimalDetectionTimestampRef.current = now;
          lastAuthorizedPetDetectedTimestampRef.current = now;
          lastAuthorizedPetNameRef.current = result.recognizedPetName || result.label;

          // PIR is deliberately evaluated by Arduino only after this AI authorization.
          // A previously reported motion event must never authorize a new feeding.
          const isPIRMotionActive = false;

          const candidatePetId = String(result.recognizedPetId || result.recognizedPetName || "");
          const samePetInPreviousFrame = candidatePetId.length > 0 && lastCandidatePetIdRef.current === candidatePetId;

          if (result.confidence >= 0.82 && consecutiveAuthorizedCountRef.current >= 1 && samePetInPreviousFrame) {
            consecutiveAuthorizedCountRef.current = 0;
            lastCandidatePetIdRef.current = "";
            verificationRetryCountRef.current = 0;
            if (unrecognizedStartTimeRef.current) {
              unrecognizedStartTimeRef.current = null;
              urgentAlertSentRef.current = false;
              setUrgentAlertActive(false);
            }

            await addAIRecognitionMutation({
              ownerId,
              speciesDetected: result.label,
              confidence: result.confidence,
              authorized: true,
              petMatch: "AUTHORIZED",
              recognizedPetId: result.recognizedPetId,
              recognizedPetName: result.recognizedPetName,
              imageUrl: capturedImageUrl,
              notificationSent: false,
            });

            if (!isPIRMotionActive) {
              sendHTTPResultToESP32(true);
              const portion = convexSettings?.foodPortion ?? 45;
              await dispenseFood(ownerId, portion, true); // Sends WAIT_MOTION, never opens the lid directly.
              triggeredFeed = true;
              addLog(`Camera recognized ${result.recognizedPetName || result.label} (${Math.round(result.confidence * 100)}%). Authorization sent to feeder. Waiting for PIR motion (10s)...`, 'info');
            } else if (isCooldown) {
              sendHTTPResultToESP32(false);
              addLog(`AI Inference: Confirmed ${result.label} + PIR Motion, BUT FEEDER IS IN COOLDOWN. Dispense BLOCKED until cooldown expires!`, 'warning');
            } else if (!isDispensing) {
              sendHTTPResultToESP32(true);
              addLog(`DUAL CONDITION CONFIRMED: Camera recognized ${result.label} (${Math.round(result.confidence * 100)}%) and the PIR sensor detected motion. Sending command through Convex Cloud to ESP8266 + Arduino...`, 'success');
              const portion = convexSettings?.foodPortion ?? 45;
              await dispenseFood(ownerId, portion, true);
              triggeredFeed = true;
            }
          } else {
            consecutiveAuthorizedCountRef.current = 1;
            lastCandidatePetIdRef.current = candidatePetId;
            verificationRetryCountRef.current = 0;
            addLog(`AI Inference: Detected ${result.label} (${Math.round(result.confidence * 100)}%). Verification 1/2 complete. Double-checking in 1s...`, 'info');

            if (nextSnapshotTimeoutRef.current) {
              clearTimeout(nextSnapshotTimeoutRef.current);
            }
            nextSnapshotTimeoutRef.current = setTimeout(() => {
              setCaptureUrl(`${baseCaptureUrl}?t=${Date.now()}`);
            }, 1000);
          }
        } else {
          consecutiveAuthorizedCountRef.current = 0;
          verificationRetryCountRef.current = 0;
          lastAuthorizedPetDetectedTimestampRef.current = 0;
          lastAuthorizedPetNameRef.current = "";
          addLog(`AI Recognition: Unauthorized subject detected: ${result.label} (${Math.round(result.confidence * 100)}%). Food access blocked.`, 'warning');

          if (result.diagnostics?.reasoning && result.label.toLowerCase().includes('unavailable')) {
            addLog(`AI diagnostic: ${result.diagnostics.reasoning}`, 'warning');
          }

          if (!isDispensing && !isFeeding) {
            sendHTTPResultToESP32(false);
          }

          await addAIRecognitionMutation({
            ownerId,
            speciesDetected: result.label,
            confidence: result.confidence,
            authorized: false,
            petMatch: result.petMatch,
            recognizedPetId: result.recognizedPetId,
            recognizedPetName: result.recognizedPetName,
            imageUrl: capturedImageUrl,
            notificationSent: false,
          });
        }
      } else {
        consecutiveAuthorizedCountRef.current = 0;
        verificationRetryCountRef.current = 0;
        lastAuthorizedPetDetectedTimestampRef.current = 0;
        lastAuthorizedPetNameRef.current = "";

        const now = Date.now();
        if (now - lastNoDetectionLogRef.current >= 10000) {
          lastNoDetectionLogRef.current = now;
          addLog("Active scan (STANDBY): No pet detected in front of the camera.", "info");
        }
      }

      if (!triggeredFeed && consecutiveAuthorizedCountRef.current === 0) {
        if (convexSettings?.deviceStatus === "PIR_MOTION") {
          await updateDeviceSettingsMutation({ ownerId, deviceStatus: "STANDBY" });
        }
      }
    } catch (err) {
      console.error('AI Recognition Error:', err);
      addLog("AI Inference Error: Failed to analyze camera snapshot.", "warning");
    } finally {
      setIsAnalyzing(false);
    }
  };

  useEffect(() => {
    triggerScanRef.current = handleTriggerScan;
  });

  useEffect(() => {
    if (isFeederBusy || !autoScanEnabled || !ownerId) {
      if (isFeederBusy && !wasFeederBusyRef.current) {
        wasFeederBusyRef.current = true;
        const reason = isFeeding ? "Feeding in progress" : isVerifying ? "Verifying motion" : "Cooldown active";
        addLog(`${reason}. AI scanning paused.`, "warning");
      }
      return;
    }

    if (wasFeederBusyRef.current) {
      wasFeederBusyRef.current = false;
      consecutiveAuthorizedCountRef.current = 0;
      addLog("Feeder ready. Resuming real-time AI scanning...", "info");
    }

    const autoScanInterval = setInterval(() => {
      if (streamError) return;
      if (snapshotMode && (aiMode === 'real' || aiMode === 'reference-test')) {
        setCaptureUrl(`${baseCaptureUrl}?t=${Date.now()}`);
      }
      if (triggerScanRef.current) {
        triggerScanRef.current();
      }
    }, scanIntervalMs);

    return () => {
      clearInterval(autoScanInterval);
      if (nextSnapshotTimeoutRef.current) {
        clearTimeout(nextSnapshotTimeoutRef.current);
      }
    };
  }, [autoScanEnabled, ownerId, isFeederBusy, scanIntervalMs, snapshotMode, aiMode, baseCaptureUrl, streamError]);

  const triggerSafetyDispense = async () => {
    if (isDispensing || !ownerId) return;
    addLog("Camera offline: feeding blocked because AI authorization is unavailable.", "warning");
    await addNotificationMutation({
      ownerId,
      title: "Feeding Blocked",
      message: "Camera failed to capture a photo. PIR motion alone cannot trigger feeding.",
      type: "Warning"
    });
  };

  useEffect(() => {
    if (convexSettings?.deviceStatus === "PIR_MOTION") {
      const now = Date.now();
      lastMotionTimestampRef.current = now;

      if (!hasProcessedMotionRef.current && !isAnalyzing && !isDispensing) {
        hasProcessedMotionRef.current = true;

        if (streamError) {
          addLog("PIR sensor detected motion, but the camera is offline. Safety lock engaged.", "warning");
          triggerSafetyDispense();
        } else {
          const timeSinceAnimal = now - lastAuthorizedPetDetectedTimestampRef.current;
          addLog(`PIR sensor detected motion. Time since AI pet recognition: ${Math.round(timeSinceAnimal / 1000)}s.`, "info");

          if (timeSinceAnimal <= 10000 && lastAuthorizedPetDetectedTimestampRef.current > 0) {
            if (isCooldown) {
              sendHTTPResultToESP32(false);
              addLog("PIR + AI confirmed the pet, but the feeder is in cooldown.", "warning");
            } else {
              addLog(`DUAL CONDITION CONFIRMED: PIR detected motion and the camera confirmed ${lastAuthorizedPetNameRef.current} ${Math.round(timeSinceAnimal / 1000)}s ago. Sending command through Convex Cloud to ESP8266 + Arduino...`, "success");
              sendHTTPResultToESP32(true);
              const portion = convexSettings?.foodPortion ?? 45;
              dispenseFood(ownerId, portion, true);
            }
          } else {
            addLog("PIR sensor detected motion. Requesting an immediate AI camera scan...", "info");
            if (triggerScanRef.current) {
              triggerScanRef.current();
            }
          }
        }
      }
    } else {
      hasProcessedMotionRef.current = false;
    }
  }, [convexSettings?.deviceStatus, streamError, isAnalyzing, isDispensing, baseCaptureUrl, ownerId, isCooldown]);

  const handleDispense = async () => {
    const activeOwner = ownerId || 'demo_owner_id';
    const portion = convexSettings?.foodPortion ?? 45;

    addLog(`MANUAL COMMAND: Starting feeding (${portion}g)...`, "success");

    // Manual requests cannot impersonate an AI authorization.

    // 2. Real Hardware Dispatch via Convex Cloud -> ESP8266 Wi-Fi Bridge -> Arduino SoftwareSerial
    addLog(`Sending hardware command through Convex Cloud to ESP8266 (pendingFeedRequest = true)...`, "info");
    const success = await dispenseFood(activeOwner, portion);

    if (success) {
      addLog(`Command sent through Convex Cloud. ESP8266 will retrieve it and send 'CMD:FORCE_FEED' to Arduino over Serial.`, "success");
    } else {
      addLog(`The command could not be sent through Convex Cloud. Check the connection.`, "warning");
    }
  };

  const historyRecords: AIRecognitionRecord[] = detections.map((item: any) => ({
    id: item._id,
    timestamp: item.timestamp,
    speciesDetected: item.speciesDetected,
    confidence: item.confidence,
    authorized: item.authorized,
    petMatch: item.petMatch,
    recognizedPetName: item.recognizedPetName,
    imageUrl: item.imageUrl,
  }));

  return (
    <div className="flex flex-col xl:flex-row min-h-[calc(100vh-72px)] p-4 md:p-8 gap-6 max-w-[1440px] mx-auto w-full">
      <div className="flex-1 flex flex-col gap-4 overflow-hidden h-full">
        {hasNoPetsRegistered && (
          <div className="w-full bg-amber-600/90 text-white p-3 rounded-2xl shadow-lg flex items-center gap-3 shrink-0">
            <span className="material-symbols-outlined">warning</span>
            <p className="text-xs font-semibold">No pets are registered. Add a pet under Registered Pets before using recognition.</p>
          </div>
        )}

        {urgentAlertActive && (
          <div className="w-full bg-red-600 text-white p-4 rounded-2xl shadow-xl flex items-center justify-between animate-pulse border-2 border-red-300 shrink-0">
            <div className="flex items-center gap-3">
              <span className="material-symbols-outlined text-3xl">error</span>
              <div>
                <h4 className="font-extrabold text-sm tracking-wide uppercase">URGENT NOTICE: FEEDING FAILED / PET NOT RECOGNIZED</h4>
                <p className="text-xs opacity-90">Two full cooldown periods passed without camera recognition. Check the live feed or dispense manually.</p>
              </div>
            </div>
            <button
              onClick={() => setUrgentAlertActive(false)}
              className="px-4 py-2 bg-white text-red-700 font-extrabold text-xs rounded-xl shadow hover:bg-gray-100 transition-all active:scale-95 shrink-0 ml-4"
            >
              DISMISS ALERT
            </button>
          </div>
        )}

        <div className="flex-1 relative bg-black rounded-3xl overflow-hidden shadow-2xl group flex items-center justify-center min-h-[360px]">
          {useDemoVideo ? (
            <video
              className="w-full h-full object-cover"
              src="https://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4"
              id="demo-video-stream"
              autoPlay
              loop
              muted
              playsInline
              crossOrigin="anonymous"
            />
          ) : aiMode !== 'real' && aiMode !== 'reference-test' ? (
            <div className="w-full h-full flex flex-col items-center justify-center bg-slate-950 text-slate-400 gap-2">
              <span className="material-symbols-outlined text-4xl text-slate-600">videocam_off</span>
              <span className="text-xs font-bold">Simulated Stream (Neutral Feed)</span>
              <img
                src="data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='320' height='240' viewBox='0 0 320 240'><rect width='320' height='240' fill='%230f172a'/><text x='50%' y='50%' dominant-baseline='middle' text-anchor='middle' fill='%23475569' font-size='14' font-family='sans-serif'>Empty Room Feed</text></svg>"
                alt="Simulated Stream Feed"
                className="w-full h-full object-cover hidden"
              />
            </div>
          ) : baseStreamUrl ? (
            <>
              <img
                id="esp32-cam-stream"
                className={streamError ? "absolute inset-0 w-0 h-0 opacity-0 pointer-events-none" : "w-full h-full object-cover"}
                src={snapshotMode ? captureUrl : baseStreamUrl}
                alt="ESP32-CAM Live Stream"
                crossOrigin="anonymous"
                onLoad={() => {
                  setStreamError(false);
                  const nowStr = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false });
                  setSnapshotTimestamp(nowStr);
                }}
                onError={() => {
                  setStreamError(true);
                  addLog("Camera communication failed. ESP32-CAM is offline or unreachable.", "warning");
                  if (convexSettings?.deviceStatus === "PIR_MOTION" && !isDispensing) {
                    triggerSafetyDispense();
                  }
                }}
              />
              {streamError && (
                <div id="stream-error-container" className="absolute inset-0 z-20 flex flex-col items-center justify-center p-6 text-center bg-black/85 backdrop-blur-md">
                  <span className="material-symbols-outlined text-amber-400 text-5xl mb-3 animate-pulse">sync</span>
                  <h4 className="text-white font-bold text-lg mb-1">Connecting to Camera...</h4>
                  <p className="text-gray-300 text-xs max-w-sm mb-4 leading-relaxed">
                    Camera stream disconnected. Guardian AI is automatically attempting to stabilize and reconnect every 5 seconds. Please wait.
                  </p>
                  <button
                    onClick={() => setCaptureUrl(`${baseCaptureUrl}?t=${Date.now()}`)}
                    className="px-4 py-2 bg-surface-container-high text-white rounded-xl text-xs font-bold hover:bg-surface-container-highest transition-all flex items-center gap-1.5 border border-outline-variant"
                  >
                    <span className="material-symbols-outlined text-sm">refresh</span>
                    Retry Now
                  </button>
                </div>
              )}
            </>
          ) : (
            <div className="flex flex-col items-center justify-center p-6 text-center text-on-surface-variant gap-2">
              <span className="material-symbols-outlined text-4xl opacity-50">videocam_off</span>
              <h4 className="font-bold text-sm text-on-surface">Camera Offline</h4>
              <p className="text-xs max-w-xs leading-relaxed opacity-75">
                ESP32-CAM stream URL not configured. Configure your camera stream URL in Settings to view live feed.
              </p>
            </div>
          )}

          <div className="absolute inset-0 z-10 flex flex-col h-full justify-between p-5 pointer-events-none">
            <div className="flex justify-between items-start w-full pointer-events-auto">
              <div className="flex gap-2">
                <div className="bg-emerald-600/90 text-white px-3 py-1.5 rounded-lg flex items-center gap-2 text-xs font-bold backdrop-blur-sm shadow-md">
                  <span className="w-1.5 h-1.5 bg-white rounded-full animate-ping"></span> SNAPSHOT MODE
                </div>
                <div className="bg-black/55 text-white px-3 py-1.5 rounded-lg text-xs font-bold backdrop-blur-sm shadow-md">
                  QVGA CAPTURE
                </div>
              </div>
              <div className="bg-black/55 text-white px-3 py-1.5 rounded-lg text-xs font-bold backdrop-blur-sm shadow-md flex items-center gap-1.5">
                <span className="material-symbols-outlined text-xs">schedule</span>
                Last Photo: {snapshotTimestamp}
              </div>
            </div>
          </div>
        </div>

        <div className="h-[220px] bg-slate-900 border border-slate-800 rounded-3xl p-5 flex flex-col shadow-lg">
          <div className="flex justify-between items-center pb-3 border-b border-slate-800 mb-3 shrink-0">
            <div className="flex items-center gap-2">
              <span className="w-2.5 h-2.5 bg-emerald-500 rounded-full animate-pulse"></span>
              <h5 className="font-bold text-xs uppercase tracking-wider text-slate-300">Camera activity log & AI announcements</h5>
            </div>
            <button
              onClick={() => setLogs([])}
              className="text-[10px] font-bold text-slate-500 hover:text-slate-300 transition-colors uppercase tracking-wider"
            >
              Clear
            </button>
          </div>

          <div className="flex-1 min-h-0 flex gap-4">
            <div ref={logContainerRef} className="flex-1 overflow-y-auto font-mono text-[11px] leading-relaxed space-y-1.5 pr-2 custom-scrollbar text-left">
              {logs.length === 0 ? (
                <div className="text-slate-600 italic text-center pt-10">No events logged. Tap "Trigger Capture" or wait for 5s...</div>
              ) : (
                logs.map((log) => (
                  <div key={log.id} className="flex gap-2.5 items-start">
                    <span className="text-slate-500 shrink-0 font-semibold">[{log.time}]</span>
                    <span className={
                      log.type === 'success' ? 'text-emerald-400 font-semibold' :
                      log.type === 'warning' ? 'text-rose-400 font-semibold' :
                      'text-slate-300'
                    }>
                      {log.text}
                    </span>
                  </div>
                ))
              )}
            </div>

            <div className="w-[200px] border-l border-slate-800 pl-4 flex flex-col justify-between shrink-0">
              <div className="space-y-2.5">
                <button
                  onClick={() => {
                    addLog("Manual scan started by user...", "info");
                    if (nextSnapshotTimeoutRef.current) clearTimeout(nextSnapshotTimeoutRef.current);
                    setCaptureUrl(`${baseCaptureUrl}?t=${Date.now()}`);
                    if (triggerScanRef.current) triggerScanRef.current();
                  }}
                  disabled={isFeederBusy}
                  className="w-full py-2.5 bg-slate-800 hover:bg-slate-750 text-white rounded-xl text-xs font-bold transition-all border border-slate-700 hover:border-slate-600 flex items-center justify-center gap-1.5 active:scale-95 disabled:opacity-50"
                >
                  <span className="material-symbols-outlined text-sm">photo_camera</span>
                  Trigger Capture
                </button>
                <button
                  onClick={handleDispense}
                  disabled={isFeederBusy || isDispensing}
                  className="w-full py-2.5 bg-emerald-600 hover:bg-emerald-500 text-white rounded-xl text-xs font-extrabold shadow-md transition-all flex items-center justify-center gap-1.5 active:scale-95 cursor-pointer disabled:opacity-50 disabled:cursor-not-allowed"
                >
                  <span className="material-symbols-outlined text-sm">restaurant</span>
                  Dispense Food (Manual)
                </button>
              </div>

              <div className="text-[10px] text-slate-500 font-medium leading-normal">
                Feeder: <span className={
                  isFeeding ? 'text-amber-400 font-bold animate-pulse' :
                  isCooldown ? 'text-blue-400 font-bold' :
                  isVerifying ? 'text-yellow-400 font-bold animate-pulse' :
                  isDispensing ? 'text-primary font-bold animate-pulse' :
                  'text-emerald-400 font-bold'
                }>
                  {isFeeding ? 'Dispensing Food' : isCooldown ? 'Cooldown Active' : isVerifying ? 'Verifying Motion' : isDispensing ? 'Sending Command' : 'Standby'}
                </span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <aside className="w-full xl:w-[400px] flex flex-col gap-6 shrink-0 h-full overflow-y-auto pr-1">
        <AIRecognitionPanel
          currentResult={aiResult}
          isAnalyzing={isAnalyzing}
          autoScanEnabled={autoScanEnabled}
          confidenceThreshold={confidenceThreshold}
          onToggleAutoScan={setAutoScanEnabled}
          onTriggerScan={handleTriggerScan}
          onChangeThreshold={setConfidenceThreshold}
          isCooldown={isCooldown}
          scanIntervalMs={scanIntervalMs}
          onChangeScanInterval={setScanIntervalMs}
        />

        <AIRecognitionHistory
          records={historyRecords}
          onClearHistory={() => ownerId && clearAIRecognitionMutation({ ownerId })}
        />
      </aside>
    </div>
  );
};

export default LiveCamera;
