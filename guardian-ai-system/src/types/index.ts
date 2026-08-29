export interface FeederSettings {
  portionWeight: number;
  cooldownMinutes: number;
  maxTemperature: number;
  initialFoodAdded: number;
  wifiSsid: string;
  wifiSignal: string;
  signalStrength: number;
  systemOnline: boolean;
  lastSyncTime: string;
  uptime: string;
  firmwareVersion: string;
  arduinoStatus: 'Active' | 'Inactive';
  nodemcuStatus: 'Connected' | 'Disconnected';
  esp32CamStatus: 'Streaming Ready' | 'Offline';
  cameraStreamUrl: string;
  notificationsEnabled: {
    feedingSuccessful: boolean;
    lowFoodWarning: boolean;
    criticalOverheat: boolean;
  };
}

export interface Pet {
  _id: string;
  id?: string;
  ownerId?: string;
  name: string;
  species: string;
  breed?: string;
  age?: string;
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
  profileImage?: string;
  trainingImages?: string[];
  aiModelStatus?: string;
  isActive?: boolean;
  createdAt?: number;
  lastSeen?: string;
  lastFeeding?: string;
  trainingPhotosCount?: number;
}

export interface AISettings {
  model1Enabled: boolean;
  model2Enabled: boolean;
  notificationsEnabled: boolean;
}
