import React from 'react';
import { useNavigate } from 'react-router-dom';
import { useUser } from '@clerk/clerk-react';
import { api } from '../../convex/_generated/api';
import { useSafeMutation, useSafeQuery } from '../lib/useSafeConvex';
import { ResponsiveContainer, AreaChart, Area, XAxis, YAxis, Tooltip, CartesianGrid } from 'recharts';

export const HealthInsights: React.FC = () => {
  const navigate = useNavigate();
  const { user } = useUser();
  const ownerId = user?.id || '';
  const feedHistory = useSafeQuery(api.feedHistory.list, { ownerId }, []) as any[];
  const deviceSettings = useSafeQuery(api.deviceSettings.get, { ownerId }, null) as any;
  const updateDeviceSettings = useSafeMutation(api.deviceSettings.update);
  const remotePets = useSafeQuery(api.pets.list, { ownerId }, []) as any[];
  let localPets: any[] = [];
  try {
    localPets = JSON.parse(localStorage.getItem('guardian_local_pets') || '[]');
  } catch {
    localPets = [];
  }
  const pets = remotePets.length > 0 ? remotePets : localPets;

  const startOfToday = new Date().setHours(0, 0, 0, 0);
  const feedingsToday = feedHistory.filter(f => f.timestamp >= startOfToday && f.completed);
  const totalToday = feedingsToday.reduce((sum, f) => sum + f.amountDispensed, 0);
  const completedFeedings = feedHistory.filter(f => f.completed);
  const totalDispensed = completedFeedings.reduce((sum, f) => sum + f.amountDispensed, 0);
  const recordedTemperatures = completedFeedings
    .map(f => Number(f.temperature))
    .filter(value => Number.isFinite(value) && value > -40 && value < 100);
  const averageTemperature = recordedTemperatures.length > 0
    ? recordedTemperatures.reduce((sum, value) => sum + value, 0) / recordedTemperatures.length
    : Number.isFinite(deviceSettings?.currentTemperature)
      ? Number(deviceSettings.currentTemperature)
      : null;

  const chartData = [...feedHistory]
    .reverse()
    .map(f => ({
      date: new Date(f.timestamp).toLocaleDateString([], { month: 'short', day: 'numeric', hour: '2-digit', minute: '2-digit' }),
      amount: f.amountDispensed,
      temperature: f.temperature,
      humidity: f.humidity,
    }));

  const initialFoodAmount = Number(deviceSettings?.initialFoodAmount ?? 0);
  const estimatedFoodRemaining = Number(deviceSettings?.estimatedFoodRemaining ?? 0);
  const remainingPercentage = initialFoodAmount > 0
    ? Math.max(0, Math.min(100, Math.round((estimatedFoodRemaining / initialFoodAmount) * 100)))
    : 0;

  const activePets = pets.filter(pet => pet.isActive !== false);
  const petKey = (pet: any) => String(pet._id || pet.id || pet.name);
  const activePetIdSignature = activePets.map(petKey).join('|');
  const [selectedPetIds, setSelectedPetIds] = React.useState<string[]>([]);
  const [expandedRecommendation, setExpandedRecommendation] = React.useState<string | null>(null);
  const [dietAction, setDietAction] = React.useState<{ petId: string; message: string; tone: 'success' | 'warning' } | null>(null);

  const programDiet = async (plan: any) => {
    const portion = Math.max(1, Math.round(plan.gramsPerMeal));
    const previousPortion = Number(deviceSettings?.foodPortion ?? 45);
    const result = await updateDeviceSettings({ ownerId, foodPortion: portion, activeDietPetId: petKey(plan.pet), activeDietPetName: plan.pet.name, activeDietMealsPerDay: plan.mealsPerDay, previousFoodPortion: previousPortion });
    if (result === null) {
      setDietAction({ petId: petKey(plan.pet), message: 'The plan could not be sent to the feeder settings.', tone: 'warning' });
      return;
    }
    setDietAction({ petId: petKey(plan.pet), message: `Plan active: ${portion} g per feeding. AI, PIR, and cooldown rules remain unchanged.`, tone: 'success' });
  };

  const cancelDiet = async (plan: any) => {
    if (deviceSettings?.activeDietPetId !== petKey(plan.pet)) return;
    const restoredPortion = Number(deviceSettings?.previousFoodPortion || 45);
    const result = await updateDeviceSettings({ ownerId, foodPortion: restoredPortion, clearActiveDiet: true });
    if (result === null) {
      setDietAction({ petId: petKey(plan.pet), message: 'Cancellation could not be saved. The feeder portion was not changed.', tone: 'warning' });
      return;
    }
    setDietAction({ petId: petKey(plan.pet), message: `Plan cancelled. The previous ${restoredPortion} g portion was restored.`, tone: 'success' });
  };

  React.useEffect(() => {
    const availableIds = activePetIdSignature ? activePetIdSignature.split('|') : [];
    setSelectedPetIds(current => {
      const validSelection = current.filter(id => availableIds.includes(id));
      return validSelection.length > 0 ? validSelection : availableIds;
    });
  }, [activePetIdSignature]);

  const selectedPets = activePets.filter(pet => selectedPetIds.includes(petKey(pet)));
  const last24Hours = Date.now() - 24 * 60 * 60 * 1000;
  const recentFailures = feedHistory.filter(feeding => feeding.timestamp >= last24Hours && !feeding.completed).length;
  const dailyGoal = selectedPets.reduce((sum, pet) => sum + Number(pet.dailyGoalGrams ?? 0), 0);
  const currentTemperature = Number.isFinite(deviceSettings?.currentTemperature)
    ? Number(deviceSettings.currentTemperature)
    : null;

  const nutritionPlans = selectedPets.map(pet => {
    const species = String(pet.species || '').toLowerCase();
    const isCat = species.includes('cat') || species.includes('pisic');
    const isDog = species.includes('dog') || species.includes('câin') || species.includes('caine');
    const currentWeight = Number(pet.weightKg || 0);
    const targetWeight = Number(pet.targetWeightKg || 0);
    const calculationWeight = targetWeight > 0 ? targetWeight : currentWeight;
    const bcs = Number(pet.bodyConditionScore || 0);
    const kcalPer100g = Number(pet.foodKcalPer100g || 0);
    const medicalConditions = String(pet.medicalConditions || '').trim();
    const hasNoDeclaredCondition = /^(none|niciuna|niciuna\.|nu|fără|fara)$/i.test(medicalConditions);
    const hasMedicalRisk = Boolean(medicalConditions && !hasNoDeclaredCondition);
    const supported = isCat || isDog;
    let factor = isCat ? (pet.neutered ? 1.2 : 1.4) : (pet.neutered ? 1.6 : 1.8);
    if (pet.lifeStage === 'growth') factor = isCat ? 2.5 : 2.0;
    if (pet.activityLevel === 'low' && pet.lifeStage !== 'growth') factor = isCat ? 1.0 : 1.4;
    if (pet.activityLevel === 'high' && pet.lifeStage !== 'growth') factor = isCat ? 1.6 : 2.0;
    const rer = calculationWeight > 0 ? 70 * Math.pow(calculationWeight, 0.75) : 0;
    const mer = rer * factor;
    const gramsPerDay = kcalPer100g > 0 ? mer / (kcalPer100g / 100) : 0;
    const mealsPerDay = pet.lifeStage === 'growth' ? 3 : 2;
    const weightDifference = targetWeight > 0 ? currentWeight - targetWeight : 0;
    const needsWeightLoss = currentWeight > 0 && targetWeight > 0 && weightDifference > 0.05;
    const needsWeightGain = currentWeight > 0 && targetWeight > currentWeight + 0.05;
    const weeklyLossMin = isCat ? 0.005 : 0.01;
    const weeklyLossMax = 0.02;
    const timeline = needsWeightLoss && supported
      ? {
          minDays: Math.ceil((weightDifference / (currentWeight * weeklyLossMax)) * 7),
          maxDays: Math.ceil((weightDifference / (currentWeight * weeklyLossMin)) * 7),
          rateLabel: isCat ? '0.5–2% of body weight per week' : '1–2% of body weight per week',
        }
      : null;
    const condition = bcs > 0 && bcs <= 3
      ? { label: 'Below ideal weight', tone: 'amber', detail: 'A medical cause should be ruled out before increasing intake.' }
      : bcs >= 4 && bcs <= 5
        ? { label: 'Ideal body condition', tone: 'emerald', detail: 'The primary goal is weight maintenance and regular monitoring.' }
        : bcs >= 6 && bcs <= 7
          ? { label: 'Above ideal weight', tone: 'amber', detail: 'Gradual, monitored weight loss is recommended without sudden restriction.' }
          : bcs >= 8
            ? { label: 'Obesity-related risk', tone: 'rose', detail: 'A veterinary assessment is recommended before starting any weight-loss diet.' }
            : { label: 'BCS not provided', tone: 'slate', detail: 'Add a Body Condition Score to interpret the goal.' };
    return {
      pet,
      supported,
      hasMedicalRisk,
      hasNoDeclaredCondition,
      rer,
      mer,
      gramsPerDay,
      mealsPerDay,
      gramsPerMeal: gramsPerDay / mealsPerDay,
      currentWeight,
      targetWeight,
      kcalPer100g,
      bcs,
      condition,
      timeline,
      needsWeightLoss,
      needsWeightGain,
      weightDifference: Math.abs(weightDifference),
      missing: [
        currentWeight <= 0 ? 'current weight' : null,
        kcalPer100g <= 0 ? 'kcal/100g from the food label' : null,
        !pet.lifeStage ? 'life stage' : null,
        !pet.activityLevel ? 'activity level' : null,
      ].filter(Boolean),
      foodGuidance: isCat
        ? `Complete and balanced cat food appropriate for the ${pet.lifeStage === 'growth' ? 'growth' : pet.lifeStage === 'senior' ? 'senior' : 'adult'} life stage.`
        : `Complete and balanced dog food appropriate for the ${pet.lifeStage === 'growth' ? 'growth' : pet.lifeStage === 'senior' ? 'senior' : 'adult'} life stage.`,
    };
  });

  const recommendations: Array<{
    icon: string;
    title: string;
    message: string;
    tone: 'info' | 'warning' | 'success';
    actionLabel: string;
    path?: string;
    details?: string;
  }> = [];

  if (activePets.length === 0) {
    recommendations.push({
      icon: 'pets',
      title: 'Complete the pet profile',
      message: 'Add a pet to receive recommendations based on breed, age, weight, and daily goal.',
      tone: 'info',
      actionLabel: 'Add pet',
      path: '/pets',
    });
  } else if (selectedPets.length === 0) {
    recommendations.push({
      icon: 'checklist',
      title: 'Select at least one pet',
      message: 'Choose the pets you want to analyze.',
      tone: 'info',
      actionLabel: 'Manage pets',
      path: '/pets',
    });
  } else {
    selectedPets.forEach(pet => {
      const profileDetails = [pet.breed, pet.age, pet.weightKg ? `${pet.weightKg} kg` : null].filter(Boolean);
      recommendations.push({
        icon: 'badge',
        title: `Monitored profile: ${pet.name}`,
        message: profileDetails.length >= 3
          ? `The analysis uses breed ${pet.breed}, age ${pet.age}, and weight ${pet.weightKg} kg.`
          : 'Add breed, age, and weight for better contextual recommendations.',
        tone: profileDetails.length >= 3 ? 'success' : 'info',
        actionLabel: profileDetails.length >= 3 ? 'View profile' : 'Complete profile',
        path: '/pets',
      });
    });

    if (dailyGoal > 0) {
      const goalRatio = totalToday / dailyGoal;
      recommendations.push(goalRatio < 0.8
        ? {
            icon: 'schedule',
            title: "Today's goal is in progress",
            message: `${totalToday}g of ${dailyGoal}g has been dispensed today. ${Math.max(0, dailyGoal - totalToday)}g remains.`,
            tone: 'info',
            actionLabel: 'View feeding history',
            path: '/history?type=feeding',
          }
        : goalRatio > 1.2
          ? {
              icon: 'trending_up',
              title: 'Amount above goal',
              message: `${totalToday}g was dispensed today, above the configured ${dailyGoal}g goal. Review portions before the next feeding.`,
              tone: 'warning',
              actionLabel: 'Adjust portion',
              path: '/settings',
            }
          : {
              icon: 'check_circle',
              title: 'Daily goal reached',
              message: `${totalToday}g was dispensed today against the configured ${dailyGoal}g goal.`,
              tone: 'success',
              actionLabel: 'View feeding history',
              path: '/history?type=feeding',
            });
    } else {
      recommendations.push({
        icon: 'target',
        title: 'Daily goal not configured',
        message: `Configure the daily goal for ${selectedPets.length === 1 ? selectedPets[0].name : 'the selected pets'} according to veterinary or food manufacturer guidance.`,
        tone: 'info',
        actionLabel: 'Configure profile',
        path: '/pets',
      });
    }
  }

  if (feedingsToday.length === 0) {
    recommendations.push({
      icon: 'history',
      title: 'No feeding recorded today',
      message: 'No completed feeding has been recorded today.',
      tone: 'info',
      actionLabel: 'Open Live Camera',
      path: '/camera',
    });
  }

  if (recentFailures > 0) {
    recommendations.push({
      icon: 'error',
      title: 'Failed feedings detected',
      message: `${recentFailures === 1 ? 'One failed attempt' : `${recentFailures} failed attempts`} in the last 24 hours. Check the mechanism and food supply.`,
      tone: 'warning',
      actionLabel: 'Check history',
      path: '/history',
    });
  }

  if (remainingPercentage > 0 && remainingPercentage <= 20) {
    recommendations.push({
      icon: 'inventory_2',
      title: 'Low food level',
      message: `Approximately ${remainingPercentage}% of food remains. A refill is recommended.`,
      tone: 'warning',
      actionLabel: 'Open settings',
      path: '/settings',
    });
  }

  if (currentTemperature !== null && currentTemperature > Number(deviceSettings?.maximumTemperature ?? 40)) {
    recommendations.push({
      icon: 'device_thermostat',
      title: 'Temperature above configured limit',
      message: `The current temperature of ${currentTemperature.toFixed(1)}°C exceeds the configured threshold. Check the device location.`,
      tone: 'warning',
      actionLabel: 'Check threshold',
      path: '/settings',
    });
  }

  return (
    <section className="p-4 md:p-8 space-y-6 max-w-[1440px] mx-auto w-full min-h-screen">

      <div className="flex justify-between items-end">
        <div>
          <h3 className="font-bold text-3xl text-on-surface tracking-tight">Health &amp; Insights</h3>
          <p className="text-sm text-on-surface-variant">Analyze pet food intake and storage metrics.</p>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
        <div className="bg-white rounded-xl p-6 shadow-sm border border-[#E2E8F0] hover:-translate-y-0.5 transition-transform duration-200">
          <div className="flex items-center justify-between mb-4">
            <div className="w-10 h-10 rounded-lg bg-primary/10 flex items-center justify-center text-primary">
              <span className="material-symbols-outlined text-lg" style={{ fontVariationSettings: "'FILL' 1" }}>monitoring</span>
            </div>
            <span className="text-primary font-semibold text-[10px] px-2 py-1 bg-primary/10 rounded-full uppercase tracking-wider">All time</span>
          </div>
          <p className="text-on-surface-variant font-medium text-xs mb-1">Total Dispensed</p>
          <h3 className="text-3xl font-bold text-on-surface">{totalDispensed}<span className="text-sm font-normal text-on-surface-variant ml-1">g</span></h3>
          <p className="text-[10px] text-on-surface-variant mt-2">{completedFeedings.length} successful feeding{completedFeedings.length === 1 ? '' : 's'}</p>
        </div>

        <div className="bg-white rounded-xl p-6 shadow-sm border border-[#E2E8F0] hover:-translate-y-0.5 transition-transform duration-200">
          <div className="flex items-center justify-between mb-4">
            <div className="w-10 h-10 rounded-lg bg-secondary/10 flex items-center justify-center text-secondary">
              <span className="material-symbols-outlined text-lg" style={{ fontVariationSettings: "'FILL' 1" }}>pets</span>
            </div>
            <span className="text-secondary font-semibold text-[10px] px-2 py-1 bg-secondary/10 rounded-full uppercase tracking-wider">Today</span>
          </div>
          <p className="text-on-surface-variant font-medium text-xs mb-1">Feedings Today</p>
          <h3 className="text-3xl font-bold text-on-surface">{feedingsToday.length}</h3>
          <p className="text-[10px] text-on-surface-variant mt-2">{totalToday}g dispensed today</p>
        </div>

        <div className="bg-white rounded-xl p-6 shadow-sm border border-[#E2E8F0] hover:-translate-y-0.5 transition-transform duration-200">
          <div className="flex items-center justify-between mb-4">
            <div className="w-10 h-10 rounded-lg bg-tertiary-container/20 flex items-center justify-center text-[#855300]">
              <span className="material-symbols-outlined text-lg">thermometer</span>
            </div>
            <span className={`font-semibold text-[10px] px-2 py-1 rounded-full uppercase tracking-wider ${averageTemperature !== null ? 'text-amber-700 bg-amber-100' : 'text-on-surface-variant bg-surface-container-high'}`}>
              {averageTemperature !== null ? 'Telemetry' : 'No data'}
            </span>
          </div>
          <p className="text-on-surface-variant font-medium text-xs mb-1">Average Temperature</p>
          <h3 className="text-3xl font-bold text-on-surface">
            {averageTemperature !== null ? averageTemperature.toFixed(1) : '—'}
            <span className="text-sm font-normal text-on-surface-variant ml-1">°C</span>
          </h3>
          <p className="text-[10px] text-on-surface-variant mt-2">{recordedTemperatures.length > 0 ? `Based on ${recordedTemperatures.length} feeding records` : 'Waiting for device telemetry'}</p>
        </div>

        <div className="bg-white rounded-xl p-6 shadow-sm border border-[#E2E8F0] hover:-translate-y-0.5 transition-transform duration-200">
          <div className="flex items-center justify-between mb-4">
            <div className="w-10 h-10 rounded-lg bg-primary-container/10 flex items-center justify-center text-primary">
              <span className="material-symbols-outlined text-lg" style={{ fontVariationSettings: "'FILL' 1" }}>inventory_2</span>
            </div>
            <span className="text-primary font-semibold text-[10px] px-2 py-1 bg-primary/10 rounded-full uppercase tracking-wider">Estimated</span>
          </div>
          <p className="text-on-surface-variant font-medium text-xs mb-1">Estimated Food Remaining</p>
          <h3 className="text-3xl font-bold text-on-surface">{estimatedFoodRemaining}<span className="text-sm font-normal text-on-surface-variant ml-1">g</span></h3>
          <div className="flex items-center gap-2 mt-3">
            <div className="flex-1 h-1.5 bg-surface-container-high rounded-full overflow-hidden">
              <div className="h-full bg-primary rounded-full" style={{ width: `${remainingPercentage}%` }}></div>
            </div>
            <span className="text-xs font-bold text-primary">{remainingPercentage}%</span>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-12 gap-6">

        <div className="col-span-12 lg:col-span-8 bg-white rounded-xl p-6 shadow-sm border border-[#E2E8F0] min-h-[350px] flex flex-col justify-between">
          <div>
            <h4 className="font-bold text-lg text-on-surface mb-1">Consumption Telemetry</h4>
            <p className="text-on-surface-variant text-xs mb-6">Food dispensed over time (grams)</p>
          </div>
          <div className="w-full h-64">
            {chartData.length > 0 ? (
              <ResponsiveContainer width="100%" height="100%">
                <AreaChart data={chartData} margin={{ top: 10, right: 10, left: -20, bottom: 0 }}>
                  <defs>
                    <linearGradient id="colorAmount" x1="0" y1="0" x2="0" y2="1">
                      <stop offset="5%" stopColor="#0F172A" stopOpacity={0.2}/>
                      <stop offset="95%" stopColor="#0F172A" stopOpacity={0}/>
                    </linearGradient>
                  </defs>
                  <CartesianGrid strokeDasharray="3 3" vertical={false} stroke="#E2E8F0" />
                  <XAxis dataKey="date" stroke="#94A3B8" fontSize={9} tickLine={false} />
                  <YAxis stroke="#94A3B8" fontSize={9} tickLine={false} />
                  <Tooltip
                    contentStyle={{ backgroundColor: '#FFF', border: '1px solid #E2E8F0', borderRadius: '8px' }}
                    labelStyle={{ fontWeight: 'bold', color: '#0F172A', fontSize: '11px' }}
                    itemStyle={{ color: '#0F172A', fontSize: '11px' }}
                  />
                  <Area type="monotone" dataKey="amount" stroke="#0F172A" strokeWidth={2} fillOpacity={1} fill="url(#colorAmount)" name="Dispensed (g)" />
                </AreaChart>
              </ResponsiveContainer>
            ) : (
              <div className="h-full flex flex-col items-center justify-center text-center space-y-3">
                <span className="material-symbols-outlined text-4xl text-outline-variant">show_chart</span>
                <h4 className="font-bold text-lg text-on-surface">No Consumption Data</h4>
                <p className="text-xs text-on-surface-variant max-w-sm mx-auto">
                  Awaiting feeding logs to display telemetry charts.
                </p>
              </div>
            )}
          </div>
        </div>

        <div className="col-span-12 lg:col-span-4 bg-white rounded-xl p-6 shadow-sm border border-[#E2E8F0] flex flex-col justify-between min-h-[350px]">
          <div>
            <h4 className="font-bold text-lg text-on-surface mb-1">Successful Feedings Today</h4>
            <p className="text-on-surface-variant text-xs mb-6">Real-time dispensing confirmation</p>
            <div className="space-y-4 max-h-60 overflow-y-auto custom-scrollbar">
              {feedingsToday.length > 0 ? (
                feedingsToday.map((f) => (
                  <div key={f._id} className="flex items-center gap-4 p-3 rounded-lg bg-surface-container-low border border-outline-variant/30">
                    <div className="w-10 h-10 rounded-full bg-primary/10 flex items-center justify-center text-primary">
                      <span className="material-symbols-outlined text-lg" style={{ fontVariationSettings: "'FILL' 1" }}>check_circle</span>
                    </div>
                    <div className="flex-1">
                      <p className="font-bold text-sm text-on-surface">{f.feedingMethod} Feeding</p>
                      <p className="text-xs text-on-surface-variant">
                        {new Date(f.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })} • {f.amountDispensed}g dispensed
                      </p>
                    </div>
                  </div>
                ))
              ) : (
                <p className="text-xs text-on-surface-variant italic text-center py-8">No feedings dispensed today.</p>
              )}
            </div>
          </div>
          <div className="mt-6 pt-4 border-t border-outline-variant/30 flex justify-between items-center">
            <span className="text-sm font-bold text-on-surface-variant">Total Today</span>
            <span className="font-bold text-xl text-primary">{totalToday}g</span>
          </div>
        </div>
      </div>

      <div className="mt-12">
        <h4 className="font-bold text-xl text-on-surface mb-6">AI Health Recommendations</h4>
        <div className="rounded-3xl border border-outline-variant/30 bg-gradient-to-br from-white to-primary/[0.03] p-6 md:p-8 shadow-sm">
          <div className="flex flex-col md:flex-row md:items-center justify-between gap-3 mb-6">
            <div>
              <p className="text-[10px] uppercase tracking-[0.2em] font-bold text-primary">Real-time local analysis</p>
              <h5 className="text-xl font-bold text-on-surface mt-1">
                {selectedPets.length === 1
                  ? `Recommendations for ${selectedPets[0].name}`
                  : selectedPets.length > 1
                    ? `Recommendations for ${selectedPets.length} pets`
                    : 'Recommendations for your pet'}
              </h5>
            </div>
            <span className="text-[10px] font-bold text-on-surface-variant bg-surface-container px-3 py-1.5 rounded-full">{recommendations.length} observations</span>
          </div>
          {activePets.length > 1 && (
            <div className="flex flex-wrap items-center gap-2 mb-6 p-3 bg-surface-container-low rounded-2xl border border-outline-variant/30">
              <span className="text-[10px] uppercase tracking-wider font-bold text-on-surface-variant mr-1">Analyze:</span>
              {activePets.map(pet => {
                const id = petKey(pet);
                const selected = selectedPetIds.includes(id);
                return (
                  <button
                    type="button"
                    key={id}
                    onClick={() => setSelectedPetIds(current =>
                      current.includes(id) ? current.filter(value => value !== id) : [...current, id]
                    )}
                    className={`inline-flex items-center gap-2 px-3 py-2 rounded-xl border text-xs font-bold transition-all ${
                      selected
                        ? 'bg-primary text-white border-primary shadow-sm'
                        : 'bg-white text-on-surface-variant border-outline-variant/40 hover:border-primary/40'
                    }`}
                  >
                    <span className="material-symbols-outlined text-[16px]">{selected ? 'check_circle' : 'circle'}</span>
                    {pet.name}
                  </button>
                );
              })}
            </div>
          )}
          <div className="grid grid-cols-1 md:grid-cols-2 gap-3">
            {recommendations.slice(0, 6).map((recommendation, index) => (
              <button
                type="button"
                onClick={() => {
                  if (recommendation.details) {
                    setExpandedRecommendation(current => current === recommendation.title ? null : recommendation.title);
                  } else if (recommendation.path) {
                    navigate(recommendation.path);
                  }
                }}
                key={`${recommendation.title}-${index}`}
                className={`group flex items-start gap-3 rounded-2xl border p-4 text-left transition-all hover:-translate-y-0.5 hover:shadow-md ${
                  recommendation.tone === 'warning'
                    ? 'bg-amber-50/70 border-amber-200/70'
                    : recommendation.tone === 'success'
                      ? 'bg-emerald-50/70 border-emerald-200/70'
                      : 'bg-surface-bright border-outline-variant/30'
                }`}
              >
                <span className={`w-9 h-9 rounded-xl flex items-center justify-center shrink-0 ${
                  recommendation.tone === 'warning'
                    ? 'bg-amber-100 text-amber-700'
                    : recommendation.tone === 'success'
                      ? 'bg-emerald-100 text-emerald-700'
                      : 'bg-primary/10 text-primary'
                }`}>
                  <span className="material-symbols-outlined text-[19px]">{recommendation.icon}</span>
                </span>
                <div className="flex-1 min-w-0">
                  <p className="font-bold text-sm text-on-surface">{recommendation.title}</p>
                  <p className="text-[11px] text-on-surface-variant leading-relaxed mt-1">{recommendation.message}</p>
                  {recommendation.details && expandedRecommendation === recommendation.title && (
                    <div className="mt-3 pt-3 border-t border-current/10 text-[11px] leading-relaxed text-on-surface">
                      {recommendation.details}
                    </div>
                  )}
                  <span className="inline-flex items-center gap-1 text-[10px] font-bold text-primary mt-3">
                    {recommendation.details
                      ? expandedRecommendation === recommendation.title ? 'Close analysis' : recommendation.actionLabel
                      : recommendation.actionLabel}
                    <span className={`material-symbols-outlined text-[14px] transition-transform ${recommendation.details && expandedRecommendation === recommendation.title ? 'rotate-180' : 'group-hover:translate-x-0.5'}`}>
                      {recommendation.details ? 'expand_more' : 'arrow_forward'}
                    </span>
                  </span>
                </div>
              </button>
            ))}
          </div>
          {nutritionPlans.length > 0 && (
            <div className="mt-7 pt-6 border-t border-outline-variant/30 space-y-4">
              <div className="rounded-2xl border border-amber-300 bg-amber-50 p-4 flex gap-3">
                <span className="material-symbols-outlined text-amber-700">medical_information</span>
                <div><p className="font-bold text-sm text-amber-950">Consult your veterinarian before starting this diet</p><p className="text-xs text-amber-900/80 mt-1">This report is an estimate, not a diagnosis, and does not automatically modify the feeder.</p></div>
              </div>
              <div>
                <p className="text-[10px] uppercase tracking-[0.2em] font-bold text-primary">Guardian Clinical Nutrition</p>
                <h6 className="text-xl font-bold text-on-surface mt-1">Individual Nutrition Report</h6>
                <p className="text-xs text-on-surface-variant mt-1">Body condition, energy requirements, and weight goals calculated from the profile.</p>
              </div>
              <div className="space-y-5">
                {nutritionPlans.map(plan => (
                  <article key={petKey(plan.pet)} className="rounded-3xl bg-white border border-outline-variant/40 p-5 md:p-6 space-y-5 shadow-sm">
                    <div className="flex items-center justify-between gap-3">
                      <div>
                        <p className="text-[10px] uppercase tracking-widest font-bold text-primary">Pacient monitorizat</p>
                        <p className="font-bold text-xl text-on-surface mt-1">{plan.pet.name}</p>
                        <p className="text-xs text-on-surface-variant">{plan.pet.species} · {plan.pet.breed || 'unspecified breed'} · {plan.pet.age || 'unspecified age'}</p>
                      </div>
                      <span className={`px-3 py-1.5 rounded-full text-[10px] font-bold uppercase ${plan.condition.tone === 'rose' ? 'bg-rose-100 text-rose-800' : plan.condition.tone === 'amber' ? 'bg-amber-100 text-amber-800' : plan.condition.tone === 'emerald' ? 'bg-emerald-100 text-emerald-800' : 'bg-slate-100 text-slate-700'}`}>
                        {plan.condition.label}
                      </span>
                    </div>

                    {plan.hasMedicalRisk ? (
                      <div className="p-3 rounded-xl bg-rose-50 border border-rose-200 text-[11px] text-rose-800">
                        The profile lists “{plan.pet.medicalConditions}”. The app does not generate a medical diet automatically; a veterinarian must establish the plan.
                      </div>
                    ) : !plan.supported ? (
                      <div className="p-3 rounded-xl bg-surface-container-low text-[11px] text-on-surface-variant">The nutrition calculator is currently available only for cats and dogs.</div>
                    ) : plan.missing.length > 0 ? (
                      <button type="button" onClick={() => navigate('/pets')} className="w-full p-3 rounded-xl bg-amber-50 border border-amber-200 text-left text-[11px] text-amber-800 hover:bg-amber-100 transition-colors">
                        Add {plan.missing.join(', ')} to calculate a portion. Open the pet profile.
                      </button>
                    ) : (
                      <div className="space-y-5">
                        <div className="grid grid-cols-2 lg:grid-cols-4 gap-3">
                          <div className="p-4 rounded-2xl bg-slate-50"><p className="text-[9px] uppercase font-bold text-on-surface-variant">Current weight</p><p className="font-bold text-lg mt-1">{plan.currentWeight} kg</p></div>
                          <div className="p-4 rounded-2xl bg-slate-50"><p className="text-[9px] uppercase font-bold text-on-surface-variant">Proposed target</p><p className="font-bold text-lg mt-1">{plan.targetWeight || '—'} kg</p></div>
                          <div className="p-4 rounded-2xl bg-slate-50"><p className="text-[9px] uppercase font-bold text-on-surface-variant">Scor corporal</p><p className="font-bold text-lg mt-1">{plan.bcs || '—'} / 9</p></div>
                          <div className="p-4 rounded-2xl bg-primary/5"><p className="text-[9px] uppercase font-bold text-on-surface-variant">Estimated portion</p><p className="font-bold text-lg text-primary mt-1">{Math.round(plan.gramsPerDay)} g/day</p></div>
                        </div>
                        <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
                          <div className="rounded-2xl border border-outline-variant/30 p-4"><p className="font-bold text-sm">Energy requirements</p><div className="grid grid-cols-2 gap-3 mt-3 text-xs"><div>RER<p className="font-bold text-sm">{Math.round(plan.rer)} kcal/day</p></div><div>Estimated MER<p className="font-bold text-sm">{Math.round(plan.mer)} kcal/day</p></div><div>Food density<p className="font-bold text-sm">{plan.kcalPer100g} kcal/100 g</p></div><div>Schedule<p className="font-bold text-sm">{Math.round(plan.gramsPerMeal)} g × {plan.mealsPerDay}</p></div></div></div>
                          <div className={`rounded-2xl border p-4 ${plan.timeline ? 'border-amber-200 bg-amber-50/60' : 'border-outline-variant/30'}`}>
                            <p className="font-bold text-sm">Weight goal</p>
                            {plan.timeline ? <><p className="text-2xl font-bold text-amber-900 mt-2">{plan.timeline.minDays}–{plan.timeline.maxDays} days</p><p className="text-xs text-amber-900/80 mt-1">Estimate for losing {plan.weightDifference.toFixed(2)} kg at {plan.timeline.rateLabel}. A veterinarian should review progress regularly.</p></> : plan.needsWeightGain ? <div className="mt-2"><p className="text-lg font-bold">Required gain: {plan.weightDifference.toFixed(2)} kg</p><p className="text-xs mt-1 text-on-surface-variant">The profile declares no known conditions. The report remains available, but a veterinarian should validate the target and rate. Recommended first reassessment: in 2–4 weeks.</p></div> : <p className="text-xs mt-2 text-on-surface-variant">Maintenance goal. Monitor weight and Body Condition Score regularly.</p>}
                          </div>
                        </div>
                        <div className="grid grid-cols-1 lg:grid-cols-2 gap-4 text-xs">
                          <div className="rounded-2xl bg-emerald-50 border border-emerald-200 p-4"><p className="font-bold text-sm text-emerald-950">Estimated meal plan</p><ul className="list-disc pl-4 mt-2 space-y-2 text-emerald-950/80"><li>{plan.foodGuidance}</li><li>Divide the ration into {plan.mealsPerDay} weighed meals and provide fresh water at all times.</li><li>Record weight regularly and adjust only after reassessment.</li>{plan.pet.allergies && !/^(none|niciuna|nu)$/i.test(plan.pet.allergies.trim()) && <li>Confirm avoidance of declared allergens: {plan.pet.allergies}.</li>}</ul></div>
                          <div className="rounded-2xl bg-rose-50 border border-rose-200 p-4"><p className="font-bold text-sm text-rose-950">Confirm with your veterinarian</p><ul className="list-disc pl-4 mt-2 space-y-2 text-rose-950/80"><li>Target weight and BCS {plan.bcs || '—'}/9 interpretation.</li><li>Calorie intake and food selection for the life stage.</li><li>Safe rate and first reassessment date.</li></ul></div>
                        </div>
                        {plan.hasNoDeclaredCondition && <div className="rounded-xl border border-sky-200 bg-sky-50 px-4 py-3 text-xs text-sky-900">The profile declares “None” under medical conditions. The calculation is available, but this declaration does not replace a veterinary assessment.</div>}
                        <div className="pt-4 border-t border-outline-variant/30 flex flex-col sm:flex-row sm:items-center justify-between gap-3">
                          <div><p className="font-bold text-sm">Feeder control</p><p className="text-[11px] text-on-surface-variant">Applies only the {Math.round(plan.gramsPerMeal)} g portion. AI, PIR, cooldown, and authorization remain active.</p></div>
                          <div className="flex flex-wrap gap-2">
                            {deviceSettings?.activeDietPetId === petKey(plan.pet) ? <button type="button" onClick={() => cancelDiet(plan)} className="px-4 py-2.5 rounded-xl border border-rose-300 text-rose-700 text-xs font-bold hover:bg-rose-50">Cancel plan</button> : <button type="button" onClick={() => programDiet(plan)} className="px-4 py-2.5 rounded-xl bg-primary text-white text-xs font-bold hover:brightness-95">Program feeder</button>}
                            <button type="button" onClick={() => navigate('/settings')} className="px-4 py-2.5 rounded-xl border border-outline-variant text-xs font-bold hover:bg-surface-container-low">Review settings</button>
                          </div>
                        </div>
                        {dietAction?.petId === petKey(plan.pet) && <div className={`rounded-xl px-4 py-3 text-xs font-medium ${dietAction.tone === 'success' ? 'bg-emerald-50 text-emerald-900 border border-emerald-200' : 'bg-amber-50 text-amber-900 border border-amber-200'}`}>{dietAction.message}</div>}
                      </div>
                    )}
                  </article>
                ))}
              </div>
            </div>
          )}
        </div>
      </div>
    </section>
  );
};

export default HealthInsights;
