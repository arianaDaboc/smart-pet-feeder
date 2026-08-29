import * as tf from '@tensorflow/tfjs';
import * as cocoSsd from '@tensorflow-models/coco-ssd';

export type PetMatchState = "AUTHORIZED" | "UNAUTHORIZED" | "UNKNOWN";

export interface RegisteredPet {
  id: string;
  name: string;
  species: string;
  breed?: string;
  profileImage?: string;
}

export interface ReferencePhoto {
  petId: string;
  name: string;
  imageBase64: string;
}

export interface AIInferenceInput {
  imageBase64?: string;
  imageElement?: HTMLImageElement | HTMLCanvasElement | HTMLVideoElement;
  registeredPets?: RegisteredPet[];
  referencePhotos?: ReferencePhoto[];
  confidenceThreshold?: number;
  aiMode?: string;
  teachableModelUrl?: string;
  qwenApiKey?: string;
}

export interface AIInferenceResult {
  detected: boolean;
  confidence: number;
  label: string;
  petMatch: PetMatchState;
  recognizedPetName?: string;
  recognizedPetId?: string;
  modelInfo: {
    name: string;
    version: string;
    inferenceTimeMs: number;
  };
  diagnostics?: {
    bestClass?: string;
    bestScore?: number;
    rawPredictionsCount?: number;
    stage?: string;
    reasoning?: string;
  };
}

export interface IAIInferenceEngine {
  recognize(input: AIInferenceInput): Promise<AIInferenceResult>;
}

function elementToBase64(el: HTMLImageElement | HTMLVideoElement | HTMLCanvasElement): string {
  if (el instanceof HTMLImageElement && el.src && el.src.startsWith('data:image/')) {
    return el.src.split(',')[1];
  }
  const canvas = document.createElement('canvas');
  let width = 320, height = 240;
  if (el instanceof HTMLVideoElement) {
    width = el.videoWidth || 320;
    height = el.videoHeight || 240;
  } else if (el instanceof HTMLImageElement) {
    width = el.naturalWidth || el.width || 320;
    height = el.naturalHeight || el.height || 240;
  } else {
    width = el.width || 320;
    height = el.height || 240;
  }
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error("Could not create canvas context");
  ctx.drawImage(el, 0, 0, width, height);
  try {
    const dataUrl = canvas.toDataURL('image/jpeg', 0.85);
    return dataUrl.split(',')[1];
  } catch (e) {
    console.warn("Canvas export failed (likely cross-origin tainted canvas). Falling back to src data:", e);
    if (el instanceof HTMLImageElement && el.src && el.src.startsWith('data:image/')) {
      return el.src.split(',')[1];
    }
    throw e;
  }
}

export class PluggableAIRecognitionEngine implements IAIInferenceEngine {
  private modelName: string = "COCO-SSD";
  private modelVersion: string = "mobilenet_v2";
  private defaultThreshold: number = 0.50;
  private cocoModel: cocoSsd.ObjectDetection | null = null;
  private isModelLoading: boolean = false;

  private async loadModel(): Promise<cocoSsd.ObjectDetection> {
    if (this.cocoModel) return this.cocoModel;
    if (this.isModelLoading) {
      while (this.isModelLoading) {
        await new Promise(resolve => setTimeout(resolve, 100));
      }
      if (this.cocoModel) return this.cocoModel;
    }
    this.isModelLoading = true;
    try {
      await tf.ready();
      this.cocoModel = await cocoSsd.load({ base: 'mobilenet_v2' });
      return this.cocoModel;
    } finally {
      this.isModelLoading = false;
    }
  }

  private matchPetToDetection(detectedClass: string, pets: RegisteredPet[]): { pet?: RegisteredPet; isAmbiguous: boolean } {
    const rawClass = (detectedClass || "").toLowerCase().trim();
    if (!rawClass || !pets || pets.length === 0) return { isAmbiguous: false };

    const catAliases = ['cat', 'pisica', 'pisică', 'motan', 'kitten', 'feline'];
    const dogAliases = ['dog', 'caine', 'câine', 'catel', 'cățel', 'pug', 'bulldog', 'puppy', 'canine'];
    const birdAliases = ['bird', 'pasare', 'pasăre', 'parrot'];

    const isCatDetection = catAliases.some(a => rawClass.includes(a));
    const isDogDetection = dogAliases.some(a => rawClass.includes(a));
    const isBirdDetection = birdAliases.some(a => rawClass.includes(a));

    const matchingPets = pets.filter(p => {
      const spec = (p.species || "").toLowerCase().trim();
      const name = (p.name || "").toLowerCase().trim();
      if (!spec && !name) return false;

      if (rawClass === spec || spec.includes(rawClass) || rawClass.includes(spec)) return true;
      if (rawClass === name || name.includes(rawClass) || rawClass.includes(name)) return true;

      const isPetCat = catAliases.some(a => spec.includes(a) || name.includes(a));
      const isPetDog = dogAliases.some(a => spec.includes(a) || name.includes(a));
      const isPetBird = birdAliases.some(a => spec.includes(a) || name.includes(a));

      if (isCatDetection && isPetCat) return true;
      if (isDogDetection && isPetDog) return true;
      if (isBirdDetection && isPetBird) return true;

      return false;
    });

    if (matchingPets.length === 0) {
      return { isAmbiguous: false };
    }

    const exactNameMatch = matchingPets.find(p => p.name.toLowerCase().trim() === rawClass);
    if (exactNameMatch) return { pet: exactNameMatch, isAmbiguous: false };

    if (matchingPets.length > 1) {
      return { pet: undefined, isAmbiguous: true };
    }

    return { pet: matchingPets[0], isAmbiguous: false };
  }

  async recognize(input: AIInferenceInput): Promise<AIInferenceResult> {
    const startTime = performance.now();
    const threshold = input.confidenceThreshold ?? this.defaultThreshold;
    const pets = input.registeredPets || [];
    const effectiveThreshold = (pets.length > 0) ? Math.min(threshold, 0.25) : threshold;

    if (!input.imageElement) return this.emptyResult(startTime);

    const img = input.imageElement;
    const isVideo = img instanceof HTMLVideoElement;
    const isLoaded = isVideo ? (img as HTMLVideoElement).readyState >= 2 : ('complete' in img ? (img as HTMLImageElement).complete : true);
    const width = isVideo ? (img as HTMLVideoElement).videoWidth : ('naturalWidth' in img ? (img as HTMLImageElement).naturalWidth : img.width);
    if (!isLoaded || width === 0) return this.emptyResult(startTime);

    try {
      const model = await this.loadModel();
      const predictions = await model.detect(img);
      const latency = Math.round(performance.now() - startTime);

      const sorted = [...predictions].sort((a, b) => b.score - a.score);
      const topPrediction = sorted[0] || null;

      if (!topPrediction || topPrediction.score < effectiveThreshold) {
        return {
          detected: false,
          confidence: topPrediction ? topPrediction.score : 0,
          label: "None",
          petMatch: "UNKNOWN",
          modelInfo: { name: this.modelName, version: this.modelVersion, inferenceTimeMs: latency },
          diagnostics: topPrediction ? { bestClass: topPrediction.class, bestScore: topPrediction.score, rawPredictionsCount: predictions.length, stage: "coco-ssd" } : undefined
        };
      }

      const rawDetectedClass = topPrediction.class.toLowerCase();
      const confidence = topPrediction.score;
      const displayLabel = rawDetectedClass.charAt(0).toUpperCase() + rawDetectedClass.slice(1);
      const { pet: matchingPet, isAmbiguous } = this.matchPetToDetection(rawDetectedClass, pets);

      if (isAmbiguous) {
        return {
          detected: true, confidence, label: `${displayLabel} (Multiple pets of this species — add a reference photo)`, petMatch: "UNAUTHORIZED",
          modelInfo: { name: this.modelName, version: this.modelVersion, inferenceTimeMs: latency },
          diagnostics: { bestClass: rawDetectedClass, bestScore: confidence, rawPredictionsCount: predictions.length, stage: "coco-ssd", reasoning: "Multiple registered pets match this species. Reference photo needed." }
        };
      }

      if (matchingPet) {
        return {

          detected: true, confidence, label: `${displayLabel} (Identity not verified)`, petMatch: "UNAUTHORIZED",
          modelInfo: { name: this.modelName, version: this.modelVersion, inferenceTimeMs: latency },
          diagnostics: { bestClass: rawDetectedClass, bestScore: confidence, rawPredictionsCount: predictions.length, stage: "coco-ssd", reasoning: "Species detection cannot prove that this is the registered pet. Visual reference verification is required." }
        };
      }

      const isPerson = rawDetectedClass === 'person';
      return {
        detected: true, confidence, label: isPerson ? "Person (Unauthorized)" : `${displayLabel} (Unauthorized)`, petMatch: "UNAUTHORIZED",
        modelInfo: { name: this.modelName, version: this.modelVersion, inferenceTimeMs: latency },
        diagnostics: { bestClass: rawDetectedClass, bestScore: confidence, rawPredictionsCount: predictions.length, stage: "coco-ssd" }
      };
    } catch (err: any) {
      const latency = Math.round(performance.now() - startTime);
      console.warn("COCO-SSD inference note:", err.message);
      return { detected: false, confidence: 0, label: "None (Scan Error)", petMatch: "UNKNOWN", modelInfo: { name: "COCO-SSD (Error)", version: this.modelVersion, inferenceTimeMs: latency } };
    }
  }

  private emptyResult(startTime: number): AIInferenceResult {
    return { detected: false, confidence: 0, label: "None", petMatch: "UNKNOWN", modelInfo: { name: this.modelName, version: this.modelVersion, inferenceTimeMs: Math.round(performance.now() - startTime) } };
  }
}

export const defaultAIEngine = new PluggableAIRecognitionEngine();

export class TeachableMachineEngine implements IAIInferenceEngine {
  private model: any = null;
  private modelUrl: string = "";

  async loadModel(url: string): Promise<any> {
    if (!url) return null;
    let cleanUrl = url.trim();
    if (!cleanUrl.endsWith('/')) cleanUrl += '/';
    if (this.model && this.modelUrl === cleanUrl) return this.model;
    try {
      const tmImage = await import('@teachablemachine/image');
      this.model = await tmImage.load(cleanUrl + 'model.json', cleanUrl + 'metadata.json');
      this.modelUrl = cleanUrl;
      return this.model;
    } catch (e) {
      console.warn("Failed to load Teachable Machine model:", e);
      this.model = null;
      return null;
    }
  }

  async recognize(input: AIInferenceInput): Promise<AIInferenceResult> {
    const startTime = performance.now();
    const pets = input.registeredPets || [];
    const threshold = input.confidenceThreshold ?? 0.60;
    if (!input.teachableModelUrl || !input.imageElement) {
      return { detected: false, confidence: 0, label: "None", petMatch: "UNKNOWN", modelInfo: { name: "Teachable Machine", version: "v1.0", inferenceTimeMs: 0 } };
    }
    const tmModel = await this.loadModel(input.teachableModelUrl);
    if (!tmModel) {
      return { detected: false, confidence: 0, label: "None (Model Load Failed)", petMatch: "UNKNOWN", modelInfo: { name: "Teachable Machine", version: "v1.0", inferenceTimeMs: Math.round(performance.now() - startTime) } };
    }
    try {
      const predictions = await tmModel.predict(input.imageElement);
      const sorted = [...predictions].sort((a: any, b: any) => b.probability - a.probability);
      const top = sorted[0];
      const latency = Math.round(performance.now() - startTime);
      if (!top || top.probability < threshold) {
        return { detected: false, confidence: top?.probability ?? 0, label: "None", petMatch: "UNKNOWN", modelInfo: { name: "Teachable Machine", version: "v1.0", inferenceTimeMs: latency } };
      }
      const detectedClassName = (top.className || "").trim();
      const lowerClass = detectedClassName.toLowerCase();
      const negativeClassNames = ['necunoscut', 'unknown', 'altul', 'altele', 'fundal', 'background', 'none', 'gol', 'empty'];
      if (negativeClassNames.some(n => lowerClass.includes(n))) {
        return { detected: false, confidence: top.probability, label: `${detectedClassName} (Empty/Negative)`, petMatch: "UNKNOWN", modelInfo: { name: "Teachable Machine", version: "v1.0", inferenceTimeMs: latency } };
      }

      const matchingPet = pets.find(p => {
        const pName = p.name.trim().toLowerCase();
        return lowerClass === pName || lowerClass.includes(pName) || pName.includes(lowerClass);
      });

      if (matchingPet) {
        return { detected: true, confidence: top.probability, label: matchingPet.name, petMatch: "AUTHORIZED", recognizedPetName: matchingPet.name, recognizedPetId: matchingPet.id, modelInfo: { name: "Teachable Machine", version: "v1.0", inferenceTimeMs: latency } };
      }
      return { detected: true, confidence: top.probability, label: `${detectedClassName} (Unauthorized)`, petMatch: "UNAUTHORIZED", modelInfo: { name: "Teachable Machine", version: "v1.0", inferenceTimeMs: latency } };
    } catch (err) {
      console.warn("Teachable Machine prediction error:", err);
      return { detected: false, confidence: 0, label: "None (Inference Error)", petMatch: "UNKNOWN", modelInfo: { name: "Teachable Machine", version: "v1.0", inferenceTimeMs: Math.round(performance.now() - startTime) } };
    }
  }
}

export const teachableAIEngine = new TeachableMachineEngine();

export class VisualReferenceAIEngine implements IAIInferenceEngine {
  async recognize(input: AIInferenceInput): Promise<AIInferenceResult> {
    const startTime = performance.now();
    const apiKey = localStorage.getItem('guardian_feeder_gemini_api_key') || localStorage.getItem('guardian_gemini_api_key');
    const refs = input.referencePhotos || [];
    const registeredPets = input.registeredPets || [];
    const petsDescription = registeredPets.map(p => `${p.name} (Specie: ${p.species || 'Cat'})`).join(", ");
    const referenceTestMode = input.aiMode === 'reference-test';

    if (!apiKey) {
      return { detected: false, confidence: 0, label: "None (API Key Missing)", petMatch: "UNKNOWN", modelInfo: { name: "Gemini 2.5 Vision", version: "v2.5-flash", inferenceTimeMs: 0 } };
    }
    if (refs.length === 0) {
      return { detected: false, confidence: 0, label: "None (Reference Photos Missing)", petMatch: "UNKNOWN", modelInfo: { name: "Gemini 2.5 Vision", version: "v2.5-flash", inferenceTimeMs: 0 }, diagnostics: { stage: "reference-validation", reasoning: "No reference photos are registered, so identity authorization is impossible." } };
    }
    if (!input.imageElement && !input.imageBase64) {
      return { detected: false, confidence: 0, label: "None", petMatch: "UNKNOWN", modelInfo: { name: "Gemini 2.5 Vision", version: "v2.5-flash", inferenceTimeMs: 0 } };
    }

    try {
      const liveBase64 = input.imageBase64 || (input.imageElement ? elementToBase64(input.imageElement) : "");

      const parts: any[] = [
        {
          text: referenceTestMode
            ? "REFERENCE TEST MODE: Registered subjects may be plush toys. Compare the LIVE plush/object directly with the labeled reference photos. If it is the same registered plush, return its exact registered name in matchedPetName. Do not reject it because it is not alive."
            : "REAL PET MODE: Authorize only a real animal matching the labeled reference photos. Plush toys and objects must remain unauthorized."
        },
        {
          text: `You are a strict AI guardian for an automatic pet feeder.
Authorized pets registered in the system: ${petsDescription || 'None'}.

CRITICAL INSTRUCTIONS:
1. Analyze the LIVE camera image. If it shows an EMPTY background (wall, floor, blurred or gray frame, with no clearly visible pet), respond exactly:
   {"detected": false, "subjectDescription": "Empty Background", "matchedPetName": null, "confidence": 0, "reasoning": "No animal in frame"}

2. If a PET appears in the image:
   - Check whether it matches one of the authorized pets (${petsDescription}).
   - Compare coat and individual features against attached reference photos.
   - If authorized, set matchedPetName to the exact registered name.
   - For an unknown pet, person, or object, set matchedPetName to null.

Return valid JSON only, without markdown (\`\`\`json):`
        },
        { inlineData: { mimeType: "image/jpeg", data: liveBase64 } }
      ];

      for (const ref of refs) {
        parts.push({ text: `REFERENCE PHOTO — registered pet name: "${ref.name}", petId: "${ref.petId}". The next image belongs to this exact pet.` });
        parts.push({ inlineData: { mimeType: "image/jpeg", data: ref.imageBase64 } });
      }

      const response = await fetch(
        `https://generativelanguage.googleapis.com/v1beta/models/gemini-2.5-flash:generateContent?key=${apiKey}`,
        {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ contents: [{ parts }] })
        }
      );

      if (!response.ok) {
        const errText = await response.text();
        throw new Error(`Gemini API Error (${response.status}): ${errText}`);
      }

      const responseData = await response.json();
      const text = responseData.candidates?.[0]?.content?.parts?.[0]?.text?.trim() || "";
      let cleanedText = text;
      if (cleanedText.startsWith("```")) {
        cleanedText = cleanedText.replace(/^```json\s*/i, "").replace(/```$/, "").trim();
      }

      const data = JSON.parse(cleanedText);
      const latency = Math.round(performance.now() - startTime);

      if (!data.detected || !data.matchedPetName) {
        if (!data.detected) {
          return {
            detected: false, confidence: data.confidence || 0, label: "None", petMatch: "UNKNOWN",
            modelInfo: { name: "Gemini 2.5 Vision", version: "v2.5-flash", inferenceTimeMs: latency },
            diagnostics: { bestClass: data.subjectDescription, bestScore: data.confidence, stage: "gemini-vision", reasoning: data.reasoning }
          };
        }
        return {
          detected: true, confidence: data.confidence || 0.8, label: `${data.subjectDescription || "Unknown"} (Unauthorized)`, petMatch: "UNAUTHORIZED",
          modelInfo: { name: "Gemini 2.5 Vision", version: "v2.5-flash", inferenceTimeMs: latency },
          diagnostics: { bestClass: data.subjectDescription, bestScore: data.confidence, stage: "gemini-vision", reasoning: data.reasoning }
        };
      }

      const normalizedMatch = String(data.matchedPetName).trim().toLowerCase();
      const matchedPet = registeredPets.find(p => p.name.trim().toLowerCase() === normalizedMatch);
      const hasReferenceForMatch = !!matchedPet && refs.some(ref => String(ref.petId) === String(matchedPet.id));
      const identityConfidence = Number(data.confidence) || 0;

      if (matchedPet && hasReferenceForMatch && identityConfidence >= 0.82) {
        return {
          detected: true, confidence: identityConfidence, label: matchedPet.name, petMatch: "AUTHORIZED",
          recognizedPetName: matchedPet.name, recognizedPetId: matchedPet.id,
          modelInfo: { name: "Gemini 2.5 Vision", version: "v2.5-flash", inferenceTimeMs: latency },
          diagnostics: { bestClass: data.subjectDescription, bestScore: data.confidence, stage: "gemini-vision", reasoning: data.reasoning }
        };
      }

      return {
        detected: true, confidence: data.confidence || 0.7, label: `${data.subjectDescription || "Unknown"} (Unauthorized)`, petMatch: "UNAUTHORIZED",
        modelInfo: { name: "Gemini 2.5 Vision", version: "v2.5-flash", inferenceTimeMs: latency },
        diagnostics: { bestClass: data.subjectDescription, bestScore: data.confidence, stage: "gemini-vision", reasoning: data.reasoning }
      };

    } catch (err: any) {
      console.error("Gemini vision compare failed:", err);
      const latency = Math.round(performance.now() - startTime);
      return { detected: false, confidence: 0, label: "None (Gemini Error)", petMatch: "UNKNOWN", modelInfo: { name: "Gemini 2.5 Vision", version: "v2.5-flash", inferenceTimeMs: latency }, diagnostics: { stage: "gemini-error", reasoning: err?.message || "Gemini request failed." } };
    }
  }
}

export const visualReferenceAIEngine = new VisualReferenceAIEngine();

export class QwenVisionAIEngine implements IAIInferenceEngine {
  async recognize(input: AIInferenceInput): Promise<AIInferenceResult> {
    const startTime = performance.now();
    const apiKey = input.qwenApiKey || localStorage.getItem('guardian_qwen_api_key') || localStorage.getItem('guardian_openrouter_api_key');
    const refs = input.referencePhotos || [];
    const registeredPets = input.registeredPets || [];
    const petsDescription = registeredPets.map(p => `${p.name} (Specie: ${p.species || 'Cat'})`).join(", ");

    if (!apiKey) {
      return { detected: false, confidence: 0, label: "None (Qwen Key Missing)", petMatch: "UNKNOWN", modelInfo: { name: "Qwen2-VL Vision", version: "2-VL", inferenceTimeMs: 0 } };
    }
    if (!input.imageElement && !input.imageBase64) {
      return { detected: false, confidence: 0, label: "None", petMatch: "UNKNOWN", modelInfo: { name: "Qwen2-VL Vision", version: "2-VL", inferenceTimeMs: 0 } };
    }

    try {
      const liveBase64 = input.imageBase64 || (input.imageElement ? elementToBase64(input.imageElement) : "");

      const promptText = `You are an advanced AI Vision Engine for an automatic pet feeder.
Analyze the LIVE camera frame and compare it with the registered authorized-pet reference photos.

Registered authorized pets:
${petsDescription || 'None (empty database)'}

CLASSIFICATION INSTRUCTIONS:
1. EMPTY FRAME:
   For a wall, floor, static background, or frame without a visible subject:
   {"detected": false, "subject": "Empty Background", "matchedPetName": null, "confidence": 0, "reasoning": "No pet in frame"}

2. PERSON WITHOUT A PET:
   For a person, human face, or hand with no pet in frame:
   {"detected": true, "subject": "Person", "matchedPetName": null, "confidence": 0.9, "reasoning": "Person detected without a pet"}

3. RECOGNIZED / AUTHORIZED PET:
   - Analyze species, coat color and pattern, ears, and face.
   - Compare against every authorized pet's reference photos (${registeredPets.map(p => p.name).join(", ")}).
   - On a visual match, set matchedPetName to that pet's exact registered name (example: "${registeredPets.length > 0 ? registeredPets[0].name : 'Luna'}").
   - For any unmatched or unauthorized pet, set matchedPetName to null.

Return exactly this JSON object:
{"detected": boolean, "subject": string, "matchedPetName": string|null, "confidence": number, "reasoning": string}`;

      const contentPayload: any[] = [
        { type: "text", text: promptText },
        { type: "image_url", image_url: { url: `data:image/jpeg;base64,${liveBase64}` } }
      ];

      for (const ref of refs) {
        contentPayload.push({ type: "text", text: `Reference photo for ${ref.name}:` });
        contentPayload.push({ type: "image_url", image_url: { url: `data:image/jpeg;base64,${ref.imageBase64}` } });
      }

      const response = await fetch("https://openrouter.ai/api/v1/chat/completions", {
        method: "POST",
        headers: {
          "Authorization": `Bearer ${apiKey}`,
          "Content-Type": "application/json",
          "HTTP-Referer": "https://guardian-ai-feeder.app",
          "X-Title": "Guardian AI Smart Feeder"
        },
        body: JSON.stringify({
          model: "qwen/qwen-2-vl-72b-instruct",
          messages: [{ role: "user", content: contentPayload }],
          temperature: 0.1,
          max_tokens: 300
        })
      });

      if (!response.ok) {
        const fallbackResponse = await fetch("https://openrouter.ai/api/v1/chat/completions", {
          method: "POST",
          headers: {
            "Authorization": `Bearer ${apiKey}`,
            "Content-Type": "application/json"
          },
          body: JSON.stringify({
            model: "qwen/qwen-2-vl-7b-instruct",
            messages: [{ role: "user", content: contentPayload }],
            temperature: 0.1
          })
        });
        if (!fallbackResponse.ok) {
          throw new Error(`Qwen API HTTP ${response.status}`);
        }
        const fbData = await fallbackResponse.json();
        return this.parseQwenResponse(fbData, input, startTime);
      }

      const resData = await response.json();
      return this.parseQwenResponse(resData, input, startTime);

    } catch (e: any) {
      console.error("Qwen VL Vision inference error:", e);
      return { detected: false, confidence: 0, label: "None (Qwen Error)", petMatch: "UNKNOWN", modelInfo: { name: "Qwen2-VL Vision", version: "2-VL", inferenceTimeMs: Math.round(performance.now() - startTime) } };
    }
  }

  private parseQwenResponse(resData: any, input: AIInferenceInput, startTime: number): AIInferenceResult {
    const latency = Math.round(performance.now() - startTime);
    const content = resData.choices?.[0]?.message?.content || "";
    let jsonStr = content.trim();
    if (jsonStr.includes("```")) {
      jsonStr = jsonStr.replace(/```json\s*/gi, "").replace(/```/g, "").trim();
    }

    try {
      const parsed = JSON.parse(jsonStr);
      const registeredPets = input.registeredPets || [];
      const refs = input.referencePhotos || [];

      if (!parsed.detected) {
        return {
          detected: false, confidence: parsed.confidence || 0, label: "None", petMatch: "UNKNOWN",
          modelInfo: { name: "Qwen2-VL Vision", version: "2-VL", inferenceTimeMs: latency },
          diagnostics: { bestClass: parsed.subject || "Empty", bestScore: parsed.confidence, stage: "qwen-vl", reasoning: parsed.reasoning || "Cadru gol" }
        };
      }

      const matchedNameStr = String(parsed.matchedPetName || '').trim().toLowerCase();
      const matchedPet = registeredPets.find(p => p.name.trim().toLowerCase() === matchedNameStr);
      const hasReferenceForMatch = !!matchedPet && refs.some(ref => String(ref.petId) === String(matchedPet.id));
      const identityConfidence = Number(parsed.confidence) || 0;

      if (matchedPet && hasReferenceForMatch && identityConfidence >= 0.82) {
          return {
            detected: true,
            confidence: identityConfidence,
            label: matchedPet.name,
            petMatch: "AUTHORIZED",
            recognizedPetName: matchedPet.name,
            recognizedPetId: matchedPet.id,
            modelInfo: { name: "Qwen2-VL Vision", version: "2-VL", inferenceTimeMs: latency },
            diagnostics: {
              bestClass: parsed.subject || "Pet",
              bestScore: parsed.confidence || 0.95,
              stage: "qwen-vl",
              reasoning: parsed.reasoning || `Recunoscut vizual ca fiind ${matchedPet.name}`
            }
          };
        }

      return {
        detected: true,
        confidence: parsed.confidence || 0.85,
        label: `${parsed.subject || "Neautorizat"} (Unauthorized)`,
        petMatch: "UNAUTHORIZED",
        modelInfo: { name: "Qwen2-VL Vision", version: "2-VL", inferenceTimeMs: latency },
        diagnostics: {
          bestClass: parsed.subject || "Unauthorized",
          bestScore: parsed.confidence || 0.85,
          stage: "qwen-vl",
          reasoning: parsed.reasoning || "Unauthorized pet or subject in frame"
        }
      };

    } catch (err) {
      console.warn("Failed to parse Qwen JSON response:", content);
      return { detected: false, confidence: 0, label: "None (Parse Error)", petMatch: "UNKNOWN", modelInfo: { name: "Qwen2-VL Vision", version: "2-VL", inferenceTimeMs: latency } };
    }
  }
}

export const qwenVisionAIEngine = new QwenVisionAIEngine();

export class LocalClipAIEngine implements IAIInferenceEngine {
  async recognize(input: AIInferenceInput): Promise<AIInferenceResult> {
    const startedAt = performance.now();
    const serverUrl = (localStorage.getItem('guardian_local_ai_url') || 'http://127.0.0.1:3000').replace(/\/$/, '');
    try {
      const base64 = input.imageBase64 || (input.imageElement ? elementToBase64(input.imageElement) : '');
      if (!base64) throw new Error('Camera frame is unavailable.');
      const imageResponse = await fetch(`data:image/jpeg;base64,${base64}`);
      const form = new FormData();
      form.append('threshold', String(input.confidenceThreshold ?? 0.88));
      form.append('image', await imageResponse.blob(), 'capture.jpg');
      const response = await fetch(`${serverUrl}/api/recognize`, { method: 'POST', body: form });
      const data = await response.json();
      if (!response.ok) throw new Error(data.error || `Local AI HTTP ${response.status}`);
      const registeredPets = input.registeredPets || [];
      const matchedPet = data.petId
        ? registeredPets.find(p => String(p.id) === String(data.petId))
        : undefined;
      const confidence = Math.max(0, Math.min(1, Number(data.similarity) || 0));
      const authorized = data.authorized === true && !!matchedPet;
      return {
        detected: true,
        confidence,
        label: authorized ? matchedPet.name : 'Unknown reference subject',
        petMatch: authorized ? 'AUTHORIZED' : 'UNAUTHORIZED',
        recognizedPetName: authorized ? matchedPet.name : undefined,
        recognizedPetId: authorized ? matchedPet.id : undefined,
        modelInfo: { name: 'Guardian Visual Identity', version: 'CLIP ViT-B/32', inferenceTimeMs: Math.round(performance.now() - startedAt) },
        diagnostics: {
          bestClass: data.pet || data.candidates?.[0]?.pet || 'Unknown',
          bestScore: confidence,
          stage: 'local-clip',
          reasoning: `similarity=${data.similarity}, threshold=${data.threshold}, margin=${data.margin}`
        }
      };
    } catch (error) {
      return {
        detected: false,
        confidence: 0,
        label: 'None (Local CLIP unavailable)',
        petMatch: 'UNKNOWN',
        modelInfo: { name: 'Guardian Visual Identity', version: 'CLIP ViT-B/32', inferenceTimeMs: Math.round(performance.now() - startedAt) },
        diagnostics: { stage: 'local-clip-error', reasoning: error instanceof Error ? error.message : 'Local AI request failed.' }
      };
    }
  }
}

export const localClipAIEngine = new LocalClipAIEngine();

export class SmartAIEngine implements IAIInferenceEngine {
  async recognize(input: AIInferenceInput): Promise<AIInferenceResult> {
    let identityFailureReason = "";
    const referenceTestMode = input.aiMode === 'reference-test';
    const geminiKey = localStorage.getItem('guardian_feeder_gemini_api_key') || localStorage.getItem('guardian_gemini_api_key');
    const qwenKey = input.qwenApiKey || localStorage.getItem('guardian_qwen_api_key') || localStorage.getItem('guardian_openrouter_api_key');

    // Registered CLIP embeddings are the primary identity check.
    const localResult = await localClipAIEngine.recognize(input);
    if (localResult.label !== 'None (Local CLIP unavailable)') return localResult;
    identityFailureReason = localResult.diagnostics?.reasoning || 'Local CLIP unavailable.';

    if (referenceTestMode) {
      if (geminiKey) {
        const result = await visualReferenceAIEngine.recognize(input);
        if (!["None (API Key Missing)", "None (Gemini Error)", "None (Reference Photos Missing)"].includes(result.label)) return result;
        identityFailureReason = `${result.label}: ${result.diagnostics?.reasoning || 'Gemini reference comparison failed.'}`;
      }
      if (qwenKey) {
        const result = await qwenVisionAIEngine.recognize(input);
        if (result.label !== "None (Qwen Key Missing)" && result.label !== "None (Qwen Error)") return result;
      }
      return {
        detected: true,
        confidence: 0,
        label: "Reference test unavailable",
        petMatch: "UNAUTHORIZED",
        modelInfo: { name: "Reference Test", version: "v1", inferenceTimeMs: 0 },
        diagnostics: { stage: "reference-test", reasoning: identityFailureReason || "Add a Gemini key and a reference photo." }
      };
    }

    const gateResult = await defaultAIEngine.recognize(input);
    const detectedClass = String(gateResult.diagnostics?.bestClass || '').toLowerCase();
    const supportedAnimalClasses = new Set([
      'cat', 'dog', 'bird', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe'
    ]);

    if (!gateResult.detected) return gateResult;

    if (!supportedAnimalClasses.has(detectedClass)) {
      return {
        ...gateResult,
        petMatch: "UNAUTHORIZED",
        label: `${gateResult.diagnostics?.bestClass || gateResult.label} (Not a live pet)`,
        diagnostics: {
          ...gateResult.diagnostics,
          stage: "animal-presence-gate",
          reasoning: "Identity comparison was not run because the local detector did not confirm a supported live-animal category."
        }
      };
    }

    if (geminiKey) {
      const geminiRes = await visualReferenceAIEngine.recognize(input);
      if (!["None (API Key Missing)", "None (Gemini Error)", "None (Reference Photos Missing)"].includes(geminiRes.label)) {
        return geminiRes;
      }
      identityFailureReason = `${geminiRes.label}: ${geminiRes.diagnostics?.reasoning || 'Gemini identity verification did not run.'}`;
    }

    if (qwenKey) {
      const qwenRes = await qwenVisionAIEngine.recognize(input);
      if (qwenRes.label !== "None (Qwen Key Missing)" && qwenRes.label !== "None (Qwen Error)") {
        return qwenRes;
      }
      identityFailureReason = `${qwenRes.label}: ${qwenRes.diagnostics?.reasoning || 'OpenRouter identity verification did not run.'}`;
    }

    if (input.teachableModelUrl) {
      const tmRes = await teachableAIEngine.recognize(input);
      if (tmRes.label !== "None (Model Load Failed)") {
        return tmRes;
      }
    }

    return {
      ...gateResult,
      petMatch: "UNAUTHORIZED",
      label: "Animal-like subject (Identity engine unavailable)",
      diagnostics: {
        ...gateResult.diagnostics,
        stage: "identity-engine-missing",
        reasoning: identityFailureReason || "Animal detected, but no working identity engine/reference dataset was available."
      }
    };
  }
}

export const smartAIEngine = new SmartAIEngine();
export const twoStageAIEngine = smartAIEngine;
export const hybridAIEngine = smartAIEngine;
