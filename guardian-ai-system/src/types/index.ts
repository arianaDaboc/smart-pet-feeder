export interface FeederSettings {
  portionWeight: number; // grams
  cooldownMinutes: number; // minutes
  maxTemperature: number; // °C
  initialFoodAdded: number; // grams
  wifiSsid: string;
  wifiSignal: string; // 'Excellent' | 'Good' | 'Fair' | 'Weak'
  signalStrength: number; // dBm
  systemOnline: boolean;
  lastSyncTime: string;
  uptime: string;
  firmwareVersion: string;
  arduinoStatus: 'Active' | 'Inactive';
  nodemcuStatus: 'Connected' | 'Disconnected';
  esp32CamStatus: 'Streaming Ready' | 'Offline';
  cameraStreamUrl: string; // ESP32-CAM stream URL
  notificationsEnabled: {
    feedingSuccessful: boolean;
    lowFoodWarning: boolean;
    criticalOverheat: boolean;
  };
}

export interface Pet {
  _id: string; // Convex document ID
  id?: string; // Compatibility alias used by local UI models
  ownerId?: string;
  name: string;
  species: string; // e.g. "Cat" | "Dog"
  breed?: string;
  age?: string; // e.g. "2y 4m"
  weightKg?: number;
  dailyGoalGrams?: number;
  lifeStage?: string;
  neutered?: boolean;
  activityLevel?: string;
  bodyConditionScore?: number;
  targetWeightKg?: number;
  foodKcalPer100g?: number;
  foodType?: string;
  allergies?: string;
  medicalConditions?: string;
  profileImage?: string; // Convex storage ID or external URL
  trainingImages?: string[];
  aiModelStatus?: string; // "Trained" | "Training" | "Not Trained"
  isActive?: boolean;
  createdAt?: number;
  lastSeen?: string; // transient UI support
  lastFeeding?: string; // transient UI support
  trainingPhotosCount?: number; // transient UI support
}

export interface AISettings {
  model1Enabled: boolean; // Cat/Dog/Human/Unknown detection
  model2Enabled: boolean; // Authorized/Unknown comparison
  notificationsEnabled: boolean;
}
