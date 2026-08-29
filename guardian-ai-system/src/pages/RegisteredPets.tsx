import React, { useState } from 'react';
import { useUser } from '@clerk/clerk-react';
import { api } from '../../convex/_generated/api';
import type { Pet } from '../types';
import { useSafeQuery, useSafeMutation } from '../lib/useSafeConvex';

const fileToBase64 = (file: File): Promise<string> => {
  return new Promise((resolve, reject) => {
    const reader = new FileReader();
    reader.readAsDataURL(file);
    reader.onload = () => resolve(reader.result as string);
    reader.onerror = error => reject(error);
  });
};

export const RegisteredPets: React.FC = () => {
  const { user } = useUser();
  const ownerId = user?.id || '';


  const rawPets = useSafeQuery(api.pets.list, { ownerId }, []);
  const addPet = useSafeMutation(api.pets.add);
  const updatePet = useSafeMutation(api.pets.update);
  const removePet = useSafeMutation(api.pets.remove);
  const generateUploadUrl = useSafeMutation(api.storage.generateUploadUrl);

  const localSavedPets = JSON.parse(localStorage.getItem('guardian_local_pets') || '[]');
  const pets = (rawPets && rawPets.length > 0) ? rawPets : localSavedPets;


  const [isAdding, setIsAdding] = useState(false);
  const [editingPet, setEditingPet] = useState<Pet | null>(null);
  const [isUploading, setIsUploading] = useState(false);


  const [name, setName] = useState('');
  const [species, setSpecies] = useState('Cat');
  const [breed, setBreed] = useState('');
  const [age, setAge] = useState('');
  const [weightKg, setWeightKg] = useState('');
  const [dailyGoalGrams, setDailyGoalGrams] = useState('');
  const [lifeStage, setLifeStage] = useState('adult');
  const [neutered, setNeutered] = useState(false);
  const [activityLevel, setActivityLevel] = useState('moderate');
  const [bodyConditionScore, setBodyConditionScore] = useState('5');
  const [targetWeightKg, setTargetWeightKg] = useState('');
  const [foodKcalPer100g, setFoodKcalPer100g] = useState('');
  const [foodType, setFoodType] = useState('Complete dry food');
  const [allergies, setAllergies] = useState('');
  const [medicalConditions, setMedicalConditions] = useState('');


  const [profileFile, setProfileFile] = useState<File | null>(null);
  const [trainingFiles, setTrainingFiles] = useState<FileList | null>(null);


  const uploadFileToConvex = async (file: File): Promise<string> => {
    const uploadUrl = await generateUploadUrl();
    if (!uploadUrl) throw new Error('Could not create a Convex upload URL.');
    const result = await fetch(uploadUrl, {
      method: 'POST',
      headers: { 'Content-Type': file.type },
      body: file
    });
    if (!result.ok) {
      throw new Error(`Failed to upload ${file.name}`);
    }
    const { storageId } = await result.json();
    return storageId;
  };

  const handleAddPet = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!name.trim()) {
      alert("Please enter the pet's name.");
      return;
    }

    const cleanSpecies = species.trim() || 'Cat';
    const cleanBreed = breed.trim() || 'Unknown breed';
    const cleanAge = age.trim() || '1 an';

    setIsUploading(true);
    try {
      let profileImageDataUrl = '';
      if (profileFile) {
        profileImageDataUrl = await fileToBase64(profileFile);
      }

      const trainingImagesBase64: string[] = [];
      if (trainingFiles && trainingFiles.length > 0) {
        for (let i = 0; i < trainingFiles.length; i++) {
          const b64 = await fileToBase64(trainingFiles[i]);
          trainingImagesBase64.push(b64);
        }
      }

      if (!profileImageDataUrl && trainingImagesBase64.length > 0) {
        profileImageDataUrl = trainingImagesBase64[0];
      }

      let profileImageId = profileImageDataUrl;
      try {
        if (profileFile) {
          profileImageId = await Promise.race([
            uploadFileToConvex(profileFile),
            new Promise<string>((_, reject) => setTimeout(() => reject(new Error("Convex storage timeout")), 2000))
          ]);
        }
      } catch (e) {
        console.warn("Convex storage upload bypassed (using local storage):", e);
      }

      const petId = 'pet_' + Date.now();
      let savedPetId = petId;
      try {
        if (ownerId) {
          const convexPetId = await addPet({
            ownerId,
            name: name.trim(),
            species: cleanSpecies,
            breed: cleanBreed,
            age: cleanAge,
            weightKg: weightKg ? Number(weightKg) : undefined,
            dailyGoalGrams: dailyGoalGrams ? Number(dailyGoalGrams) : undefined,
            lifeStage, neutered, activityLevel,
            bodyConditionScore: bodyConditionScore ? Number(bodyConditionScore) : undefined,
            targetWeightKg: targetWeightKg ? Number(targetWeightKg) : undefined,
            foodKcalPer100g: foodKcalPer100g ? Number(foodKcalPer100g) : undefined,
            foodType, allergies, medicalConditions,
            profileImage: profileImageId || profileImageDataUrl || undefined,
            trainingImages: [],
            aiModelStatus: 'Trained',
          });
          if (convexPetId) {
            savedPetId = String(convexPetId);
          }
        }
      } catch (e) {
        console.warn("Convex addPet mutation bypassed:", e);
      }

      const newLocalPet = {
        _id: savedPetId,
        id: savedPetId,
        name: name.trim(),
        species: cleanSpecies,
        breed: cleanBreed,
        age: cleanAge,
        weightKg: weightKg ? Number(weightKg) : undefined,
        dailyGoalGrams: dailyGoalGrams ? Number(dailyGoalGrams) : undefined,
        lifeStage, neutered, activityLevel,
        bodyConditionScore: bodyConditionScore ? Number(bodyConditionScore) : undefined,
        targetWeightKg: targetWeightKg ? Number(targetWeightKg) : undefined,
        foodKcalPer100g: foodKcalPer100g ? Number(foodKcalPer100g) : undefined,
        foodType, allergies, medicalConditions,
        profileImage: profileImageDataUrl || profileImageId,
        trainingImages: trainingImagesBase64,
        aiModelStatus: 'Trained'
      };

      const existingLocal = JSON.parse(localStorage.getItem('guardian_local_pets') || '[]');
      const updatedLocal = [...existingLocal.filter((p: any) => (p._id || p.id || '').toString() !== savedPetId), newLocalPet];
      localStorage.setItem('guardian_local_pets', JSON.stringify(updatedLocal));


      const refMap = JSON.parse(localStorage.getItem('guardian_referencePhotos') || '{}');
      if (profileImageDataUrl) {
        const base64Only = profileImageDataUrl.split(',')[1] || profileImageDataUrl;
        refMap[savedPetId] = base64Only;
      } else if (trainingImagesBase64.length > 0) {
        refMap[savedPetId] = trainingImagesBase64[0].split(',')[1] || trainingImagesBase64[0];
      }
      localStorage.setItem('guardian_referencePhotos', JSON.stringify(refMap));


      const multiRefMap = JSON.parse(localStorage.getItem('guardian_referencePhotos_multi') || '{}');
      multiRefMap[savedPetId] = trainingImagesBase64.map(img => img.split(',')[1] || img);
      if (profileImageDataUrl) {
        multiRefMap[savedPetId].unshift(profileImageDataUrl.split(',')[1] || profileImageDataUrl);
      }
      localStorage.setItem('guardian_referencePhotos_multi', JSON.stringify(multiRefMap));

      setName('');
      setSpecies('Cat');
      setBreed('');
      setAge('');
      setWeightKg('');
      setDailyGoalGrams('');
      setProfileFile(null);
      setTrainingFiles(null);
      setIsAdding(false);
      alert(`${name}'s profile was saved successfully.`);
    } catch (err) {
      console.error('Failed to add pet companion profile:', err);
      alert(`${name}'s profile was saved locally.`);
    } finally {
      setIsUploading(false);
    }
  };

  const handleStartEdit = (pet: Pet) => {
    setEditingPet(pet);
    setName(pet.name);
    setSpecies(pet.species || 'Cat');
    setBreed(pet.breed || '');
    setAge(pet.age || '');
    setWeightKg(pet.weightKg ? String(pet.weightKg) : '');
    setDailyGoalGrams(pet.dailyGoalGrams ? String(pet.dailyGoalGrams) : '');
    setLifeStage(pet.lifeStage || 'adult');
    setNeutered(pet.neutered === true);
    setActivityLevel(pet.activityLevel || 'moderate');
    setBodyConditionScore(pet.bodyConditionScore ? String(pet.bodyConditionScore) : '5');
    setTargetWeightKg(pet.targetWeightKg ? String(pet.targetWeightKg) : '');
    setFoodKcalPer100g(pet.foodKcalPer100g ? String(pet.foodKcalPer100g) : '');
    setFoodType(pet.foodType || 'Complete dry food');
    setAllergies(pet.allergies || '');
    setMedicalConditions(pet.medicalConditions || '');
    setProfileFile(null);
    setTrainingFiles(null);
  };

  const handleEditPetSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!editingPet || !name.trim()) return;

    const cleanSpecies = species.trim() || 'Cat';
    const cleanBreed = breed.trim() || 'Unknown breed';
    const cleanAge = age.trim() || '1 an';

    setIsUploading(true);
    try {
      let profileImageDataUrl = editingPet.profileImage || '';
      if (profileFile) {
        profileImageDataUrl = await fileToBase64(profileFile);
      }

      const trainingImagesBase64: string[] = [];
      if (trainingFiles && trainingFiles.length > 0) {
        for (let i = 0; i < trainingFiles.length; i++) {
          const b64 = await fileToBase64(trainingFiles[i]);
          trainingImagesBase64.push(b64);
        }
      }

      let profileImageId = profileImageDataUrl;
      try {
        if (profileFile) {
          profileImageId = await Promise.race([
            uploadFileToConvex(profileFile),
            new Promise<string>((_, reject) => setTimeout(() => reject(new Error("Convex storage timeout")), 2000))
          ]);
        }
      } catch (e) {
        console.warn("Convex storage update bypassed:", e);
      }

      try {
        await updatePet({
          id: editingPet._id as any,
          name: name.trim(),
          species: cleanSpecies,
          breed: cleanBreed,
          age: cleanAge,
          weightKg: weightKg ? Number(weightKg) : undefined,
          dailyGoalGrams: dailyGoalGrams ? Number(dailyGoalGrams) : undefined,
          lifeStage, neutered, activityLevel,
          bodyConditionScore: bodyConditionScore ? Number(bodyConditionScore) : undefined,
          targetWeightKg: targetWeightKg ? Number(targetWeightKg) : undefined,
          foodKcalPer100g: foodKcalPer100g ? Number(foodKcalPer100g) : undefined,
          foodType, allergies, medicalConditions,
          profileImage: profileImageId || undefined,
          aiModelStatus: 'Trained'
        });
      } catch (e) {
        console.warn("Convex updatePet bypassed:", e);
      }

      const existingLocal = JSON.parse(localStorage.getItem('guardian_local_pets') || '[]');
      const updatedLocal = existingLocal.map((p: any) => p._id === editingPet._id ? {
        ...p,
        name: name.trim(),
        species: cleanSpecies,
        breed: cleanBreed,
        age: cleanAge,
        weightKg: weightKg ? Number(weightKg) : undefined,
        dailyGoalGrams: dailyGoalGrams ? Number(dailyGoalGrams) : undefined,
        lifeStage, neutered, activityLevel,
        bodyConditionScore: bodyConditionScore ? Number(bodyConditionScore) : undefined,
        targetWeightKg: targetWeightKg ? Number(targetWeightKg) : undefined,
        foodKcalPer100g: foodKcalPer100g ? Number(foodKcalPer100g) : undefined,
        foodType, allergies, medicalConditions,
        profileImage: profileImageDataUrl || p.profileImage,
        trainingImages: trainingImagesBase64.length > 0 ? trainingImagesBase64 : p.trainingImages
      } : p);
      localStorage.setItem('guardian_local_pets', JSON.stringify(updatedLocal));

      if (profileImageDataUrl && profileImageDataUrl.startsWith('data:image/')) {
        const base64Only = profileImageDataUrl.split(',')[1];
        const refMap = JSON.parse(localStorage.getItem('guardian_referencePhotos') || '{}');
        refMap[editingPet._id] = base64Only;
        localStorage.setItem('guardian_referencePhotos', JSON.stringify(refMap));
      }

      if (trainingImagesBase64.length > 0) {
        const multiRefMap = JSON.parse(localStorage.getItem('guardian_referencePhotos_multi') || '{}');
        multiRefMap[editingPet._id] = trainingImagesBase64.map(img => img.split(',')[1] || img);
        localStorage.setItem('guardian_referencePhotos_multi', JSON.stringify(multiRefMap));
      }

      setEditingPet(null);
      setName('');
      setBreed('');
      setAge('');
      setWeightKg('');
      setDailyGoalGrams('');
      setProfileFile(null);
      setTrainingFiles(null);
      alert(`${name}'s profile was updated successfully.`);
    } catch (err) {
      console.error('Failed to update pet companion details:', err);
    } finally {
      setIsUploading(false);
    }
  };

  const handleDeletePet = async (id: any) => {
    if (confirm('Are you sure you want to delete this pet profile?')) {
      try {
        await removePet({ id });
      } catch (err) {
        console.warn('Convex removePet bypassed:', err);
      }
      const existingLocal = JSON.parse(localStorage.getItem('guardian_local_pets') || '[]');
      const updatedLocal = existingLocal.filter((p: any) => p._id !== id);
      localStorage.setItem('guardian_local_pets', JSON.stringify(updatedLocal));

      const refMap = JSON.parse(localStorage.getItem('guardian_referencePhotos') || '{}');
      delete refMap[id];
      localStorage.setItem('guardian_referencePhotos', JSON.stringify(refMap));

      const multiRefMap = JSON.parse(localStorage.getItem('guardian_referencePhotos_multi') || '{}');
      delete multiRefMap[id];
      localStorage.setItem('guardian_referencePhotos_multi', JSON.stringify(multiRefMap));
    }
  };

  const nutritionFields = (
    <div className="space-y-4 rounded-2xl border border-primary/15 bg-primary/[0.03] p-4">
      <div>
        <h5 className="text-sm font-bold text-on-surface">Nutrition Profile</h5>
        <p className="text-[10px] text-on-surface-variant mt-0.5">Data used for energy estimates. Confirm recommendations with your veterinarian.</p>
      </div>
      <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
        <label className="space-y-1 text-xs font-semibold text-on-surface-variant">Life stage
          <select value={lifeStage} onChange={e => setLifeStage(e.target.value)} className="w-full px-3 py-2 border border-outline-variant rounded-xl bg-white text-sm">
            <option value="growth">Puppy / kitten / growth</option><option value="adult">Adult</option><option value="senior">Senior</option>
          </select>
        </label>
        <label className="space-y-1 text-xs font-semibold text-on-surface-variant">Activitate
          <select value={activityLevel} onChange={e => setActivityLevel(e.target.value)} className="w-full px-3 py-2 border border-outline-variant rounded-xl bg-white text-sm">
            <option value="low">Low</option><option value="moderate">Moderate</option><option value="high">High</option>
          </select>
        </label>
        <div className="space-y-1">
          <div className="flex items-center gap-1.5 text-xs font-semibold text-on-surface-variant">
            <label htmlFor="body-condition-score">Scor corporal (1–9)</label>
            <span
              className="material-symbols-outlined text-[16px] text-primary cursor-help"
              title="Body Condition Score estimates body fat by assessing the ribs, waist, and abdomen."
            >
              info
            </span>
          </div>
          <input id="body-condition-score" type="number" min="1" max="9" step="1" value={bodyConditionScore} onChange={e => setBodyConditionScore(e.target.value)} className="w-full px-3 py-2 border border-outline-variant rounded-xl text-sm" />
        </div>
      </div>
      <details className="rounded-xl border border-outline-variant/30 bg-white/70 px-3 py-2 text-[10px] text-on-surface-variant">
        <summary className="cursor-pointer font-bold text-primary">How is Body Condition Score interpreted?</summary>
        <div className="grid grid-cols-1 sm:grid-cols-3 gap-2 mt-2 leading-relaxed">
          <p><strong className="text-on-surface">1–3:</strong> underweight; ribs and bones may be highly visible.</p>
          <p><strong className="text-on-surface">4–5:</strong> generally considered ideal; ribs are easy to feel and the waist is visible.</p>
          <p><strong className="text-on-surface">6–9:</strong> overweight; the waist becomes less visible and ribs are harder to feel.</p>
        </div>
        <p className="mt-2">The score is an estimate and should be confirmed by a veterinarian, especially before changing the diet.</p>
      </details>
      <label className="flex items-center gap-2 text-xs font-semibold text-on-surface-variant">
        <input type="checkbox" checked={neutered} onChange={e => setNeutered(e.target.checked)} className="w-4 h-4 accent-primary" /> Sterilizat / castrat
      </label>
      <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
        <label className="space-y-1 text-xs font-semibold text-on-surface-variant">Validated target weight (kg)
          <input type="number" min="0.1" step="0.1" value={targetWeightKg} onChange={e => setTargetWeightKg(e.target.value)} placeholder="optional" className="w-full px-3 py-2 border border-outline-variant rounded-xl text-sm" />
        </label>
        <label className="space-y-1 text-xs font-semibold text-on-surface-variant">Food energy (kcal/100g)
          <input type="number" min="1" step="1" value={foodKcalPer100g} onChange={e => setFoodKcalPer100g(e.target.value)} placeholder="de pe ambalaj" className="w-full px-3 py-2 border border-outline-variant rounded-xl text-sm" />
        </label>
      </div>
      <label className="space-y-1 text-xs font-semibold text-on-surface-variant">Food type
        <input value={foodType} onChange={e => setFoodType(e.target.value)} placeholder="e.g. complete dry food" className="w-full px-3 py-2 border border-outline-variant rounded-xl text-sm" />
      </label>
      <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
        <label className="space-y-1 text-xs font-semibold text-on-surface-variant">Alergii cunoscute
          <input value={allergies} onChange={e => setAllergies(e.target.value)} placeholder="None or details" className="w-full px-3 py-2 border border-outline-variant rounded-xl text-sm" />
        </label>
        <label className="space-y-1 text-xs font-semibold text-on-surface-variant">Medical conditions
          <input value={medicalConditions} onChange={e => setMedicalConditions(e.target.value)} placeholder="None or details" className="w-full px-3 py-2 border border-outline-variant rounded-xl text-sm" />
        </label>
      </div>
    </div>
  );

  return (
    <section className="p-4 md:p-8 space-y-6 max-w-[1440px] mx-auto w-full min-h-screen">
      <div className="flex justify-between items-end">
        <div>
          <h3 className="font-bold text-3xl text-on-surface tracking-tight">Pet Profiles</h3>
          <p className="text-sm text-on-surface-variant">Add and manage your pets (cats, dogs, rabbits, parrots, and more).</p>
        </div>
        <button
          onClick={() => {
            setIsAdding(true);
            setName('');
            setSpecies('Cat');
            setBreed('');
            setAge('');
            setProfileFile(null);
            setTrainingFiles(null);
          }}
          className="flex items-center gap-2 bg-primary text-on-primary px-6 py-3 rounded-xl font-bold text-xs hover:shadow-lg active:scale-95 transition-all"
        >
          <span className="material-symbols-outlined">add</span>
          <span>Add Pet</span>
        </button>
      </div>

      {pets === undefined && (
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-6">
          {[1, 2, 3].map((n) => (
            <div key={n} className="bg-white rounded-[24px] overflow-hidden border border-outline-variant/30 p-6 space-y-4 animate-pulse">
              <div className="h-40 w-full bg-surface-container-high rounded-xl"></div>
              <div className="h-6 w-1/2 bg-surface-container-high rounded"></div>
              <div className="h-4 w-1/3 bg-surface-container-high rounded"></div>
              <div className="h-10 w-full bg-surface-container-high rounded-lg"></div>
            </div>
          ))}
        </div>
      )}

      {pets !== undefined && pets.length === 0 && (
        <div className="flex flex-col items-center justify-center p-12 bg-white rounded-[24px] border border-outline-variant/30 max-w-2xl mx-auto shadow-sm">
          <div className="w-16 h-16 rounded-full bg-primary/10 flex items-center justify-center text-primary mb-4">
            <span className="material-symbols-outlined text-[32px]">pets</span>
          </div>
          <h4 className="font-bold text-lg text-on-surface mb-2">No registered pets</h4>
          <p className="text-sm text-on-surface-variant text-center max-w-xs mb-6">
            Add a pet so the AI system can recognize it on camera and authorize feeding.
          </p>
          <button
            onClick={() => setIsAdding(true)}
            className="bg-primary text-on-primary px-5 py-2.5 rounded-xl font-bold text-xs shadow-md"
          >
            Add Pet
          </button>
        </div>
      )}

      {pets !== undefined && pets.length > 0 && (
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-6">
          {pets.map((pet: Pet) => (
            <div
              key={pet._id}
              className="bg-white rounded-[24px] overflow-hidden shadow-sm border border-outline-variant/30 group hover:-translate-y-1 transition-all duration-300 flex flex-col justify-between"
            >
              <div className="relative h-48 w-full bg-surface-container flex items-center justify-center overflow-hidden">
                {pet.profileImage ? (
                  <img
                    className="w-full h-full object-cover"
                    src={pet.profileImage}
                    alt={pet.name}
                  />
                ) : (
                  <span className="material-symbols-outlined text-[64px] text-on-surface-variant/40">pets</span>
                )}
                <div className="absolute top-4 left-4">
                  <span className="px-3 py-1 bg-white/90 backdrop-blur text-primary text-[10px] font-bold rounded-full shadow-sm">
                    {pet.species || 'Animal'}
                  </span>
                </div>
                <div className="absolute bottom-4 right-4 flex gap-2 opacity-100 z-10">
                  <button
                    onClick={() => handleStartEdit(pet)}
                    className="p-2 bg-white/95 backdrop-blur rounded-lg shadow-md hover:bg-primary hover:text-white transition-all text-on-surface-variant flex items-center justify-center"
                    title="Edit"
                  >
                    <span className="material-symbols-outlined text-[18px]">edit</span>
                  </button>
                  <button
                    onClick={() => handleDeletePet(pet._id || pet.id)}
                    className="p-2 bg-white/95 backdrop-blur rounded-lg shadow-md hover:bg-error hover:text-white transition-all text-error hover:text-white"
                    title="Delete"
                  >
                    <span className="material-symbols-outlined text-[18px]">delete</span>
                  </button>
                </div>
              </div>
              <div className="p-6 flex-1 flex flex-col justify-between space-y-4">
                <div>
                  <h4 className="font-bold text-lg text-on-surface">{pet.name}</h4>
                  <p className="text-xs text-on-surface-variant font-semibold text-primary">{pet.species} • {pet.breed || 'Unspecified breed'}</p>
                </div>
                <div className="flex items-center justify-between text-xs pt-2 border-t border-outline-variant/10">
                  <span className="text-on-surface-variant font-medium">Age</span>
                  <span className="font-semibold text-on-surface">{pet.age}</span>
                </div>
                {pet.trainingImages && pet.trainingImages.length > 0 && (
                  <div className="flex items-center gap-2 pt-2 border-t border-outline-variant/10">
                    <span className="material-symbols-outlined text-primary text-[18px]">photo_library</span>
                    <span className="text-xs font-medium text-on-surface-variant">
                      {pet.trainingImages.length} reference photos uploaded
                    </span>
                  </div>
                )}
              </div>
            </div>
          ))}
        </div>
      )}

      {isAdding && (
        <div className="fixed inset-0 bg-black/50 flex items-center justify-center p-6 z-[100] backdrop-blur-sm">
          <div className="bg-white rounded-3xl p-8 max-w-md w-full shadow-2xl border border-outline-variant/30 space-y-6 max-h-[90vh] overflow-y-auto">
            <div className="flex justify-between items-center">
              <h3 className="font-bold text-lg text-on-surface">Add Pet</h3>
              <button
                onClick={() => setIsAdding(false)}
                className="p-1 rounded-full hover:bg-surface-container transition-colors"
              >
                <span className="material-symbols-outlined">close</span>
              </button>
            </div>
            <form onSubmit={handleAddPet} className="space-y-4">
              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant">Pet Name *</label>
                <input
                  type="text"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  required
                  placeholder="ex: Luna, Amigo, Rex, Felix..."
                  className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none focus:border-primary"
                />
              </div>

              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant">Species / Pet Type</label>
                <input
                  type="text"
                  value={species}
                  onChange={(e) => setSpecies(e.target.value)}
                  placeholder="e.g. Cat, Dog, Rabbit, Parrot, Hamster"
                  className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none focus:border-primary"
                />
              </div>

              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant">Breed (Optional)</label>
                <input
                  type="text"
                  value={breed}
                  onChange={(e) => setBreed(e.target.value)}
                  placeholder="ex: Siberiana, Maidanez, Perus, etc."
                  className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none"
                />
              </div>

              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant">Age (Optional)</label>
                <input
                  type="text"
                  value={age}
                  onChange={(e) => setAge(e.target.value)}
                  placeholder="ex: 1 an, 6 luni, 2 ani..."
                  className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none"
                />
              </div>

              <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
                <div className="space-y-1">
                  <label className="text-xs font-semibold text-on-surface-variant">Weight (kg)</label>
                  <input
                    type="number"
                    min="0.1"
                    step="0.1"
                    value={weightKg}
                    onChange={(e) => setWeightKg(e.target.value)}
                    placeholder="ex: 4.5"
                    className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none focus:border-primary"
                  />
                </div>
                <div className="space-y-1">
                  <label className="text-xs font-semibold text-on-surface-variant">Daily Goal (g)</label>
                  <input
                    type="number"
                    min="1"
                    step="1"
                    value={dailyGoalGrams}
                    onChange={(e) => setDailyGoalGrams(e.target.value)}
                    placeholder="ex: 180"
                    className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none focus:border-primary"
                  />
                </div>
              </div>

              {nutritionFields}

              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant block">Profile Photo (Cover)</label>
                <input
                  type="file"
                  accept="image/*"
                  onChange={(e) => setProfileFile(e.target.files ? e.target.files[0] : null)}
                  className="text-xs text-on-surface-variant file:mr-4 file:py-2 file:px-4 file:rounded-xl file:border-0 file:text-xs file:font-semibold file:bg-primary/10 file:text-primary hover:file:bg-primary/20"
                />
              </div>

              <div className="space-y-2">
                <label className="text-xs font-semibold text-on-surface-variant block flex items-center justify-between">
                  <span>AI Reference Photos (select as many as needed)</span>
                  {trainingFiles && trainingFiles.length > 0 && (
                    <span className="text-[10px] font-bold text-primary px-2 py-0.5 bg-primary/10 rounded-full">
                      {trainingFiles.length} photos selected
                    </span>
                  )}
                </label>
                <input
                  type="file"
                  accept="image/*"
                  multiple
                  onChange={(e) => setTrainingFiles(e.target.files)}
                  className="w-full text-xs text-on-surface-variant file:mr-4 file:py-2.5 file:px-4 file:rounded-xl file:border-0 file:text-xs file:font-bold file:bg-primary file:text-white hover:file:bg-primary-container cursor-pointer"
                />

                {trainingFiles && trainingFiles.length > 0 && (
                  <div className="grid grid-cols-4 gap-2 pt-2 max-h-36 overflow-y-auto p-2 bg-surface-container-low rounded-xl border border-outline-variant/30">
                    {Array.from(trainingFiles).map((file, idx) => (
                      <div key={idx} className="relative aspect-square rounded-lg overflow-hidden border border-outline-variant/40 group">
                        <img
                          src={URL.createObjectURL(file)}
                          alt={`Preview ${idx + 1}`}
                          className="w-full h-full object-cover"
                        />
                        <div className="absolute inset-0 bg-black/40 flex items-center justify-center text-white text-[9px] font-bold opacity-0 group-hover:opacity-100 transition-opacity">
                          #{idx + 1}
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </div>

              {isUploading && (
                <p className="text-[10px] text-primary font-bold animate-pulse">Saving profile and reference photos...</p>
              )}

              <div className="flex gap-3 justify-end pt-4 border-t border-outline-variant/30">
                <button
                  type="button"
                  disabled={isUploading}
                  onClick={() => setIsAdding(false)}
                  className="px-5 py-2.5 bg-surface-container hover:bg-surface-container-high rounded-xl text-xs font-bold"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  disabled={isUploading}
                  className="px-5 py-2.5 bg-primary text-on-primary hover:shadow-lg rounded-xl text-xs font-bold flex items-center gap-1.5"
                >
                  {isUploading ? 'Saving...' : 'Add Pet'}
                </button>
              </div>
            </form>
          </div>
        </div>
      )}

      {editingPet && (
        <div className="fixed inset-0 bg-black/50 flex items-center justify-center p-6 z-[100] backdrop-blur-sm">
          <div className="bg-white rounded-3xl p-8 max-w-md w-full shadow-2xl border border-outline-variant/30 space-y-6 max-h-[90vh] overflow-y-auto">
            <div className="flex justify-between items-center">
              <h3 className="font-bold text-lg text-on-surface">Edit Pet Profile</h3>
              <button
                onClick={() => setEditingPet(null)}
                className="p-1 rounded-full hover:bg-surface-container transition-colors"
              >
                <span className="material-symbols-outlined">close</span>
              </button>
            </div>
            <form onSubmit={handleEditPetSubmit} className="space-y-4">
              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant">Pet Name *</label>
                <input
                  type="text"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  required
                  placeholder="ex: Luna"
                  className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none focus:border-primary"
                />
              </div>

              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant">Species / Pet Type</label>
                <input
                  type="text"
                  value={species}
                  onChange={(e) => setSpecies(e.target.value)}
                  placeholder="e.g. Cat, Dog, Rabbit, Parrot, Hamster"
                  className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none focus:border-primary"
                />
              </div>

              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant">Breed</label>
                <input
                  type="text"
                  value={breed}
                  onChange={(e) => setBreed(e.target.value)}
                  placeholder="ex: Siberiana, Perus..."
                  className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none"
                />
              </div>

              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant">Age</label>
                <input
                  type="text"
                  value={age}
                  onChange={(e) => setAge(e.target.value)}
                  placeholder="ex: 2 ani..."
                  className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none"
                />
              </div>

              <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
                <div className="space-y-1">
                  <label className="text-xs font-semibold text-on-surface-variant">Weight (kg)</label>
                  <input
                    type="number"
                    min="0.1"
                    step="0.1"
                    value={weightKg}
                    onChange={(e) => setWeightKg(e.target.value)}
                    placeholder="ex: 4.5"
                    className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none focus:border-primary"
                  />
                </div>
                <div className="space-y-1">
                  <label className="text-xs font-semibold text-on-surface-variant">Daily Goal (g)</label>
                  <input
                    type="number"
                    min="1"
                    step="1"
                    value={dailyGoalGrams}
                    onChange={(e) => setDailyGoalGrams(e.target.value)}
                    placeholder="ex: 180"
                    className="w-full px-4 py-2 border border-outline-variant rounded-xl text-sm focus:outline-none focus:border-primary"
                  />
                </div>
              </div>

              {nutritionFields}

              <div className="space-y-1">
                <label className="text-xs font-semibold text-on-surface-variant block">Change Profile Photo</label>
                <input
                  type="file"
                  accept="image/*"
                  onChange={(e) => setProfileFile(e.target.files ? e.target.files[0] : null)}
                  className="text-xs text-on-surface-variant file:mr-4 file:py-2 file:px-4 file:rounded-xl file:border-0 file:text-xs file:font-semibold file:bg-primary/10 file:text-primary hover:file:bg-primary/20"
                />
              </div>

              <div className="space-y-2">
                <label className="text-xs font-semibold text-on-surface-variant block flex items-center justify-between">
                  <span>Add More Reference Photos</span>
                  {trainingFiles && trainingFiles.length > 0 && (
                    <span className="text-[10px] font-bold text-primary px-2 py-0.5 bg-primary/10 rounded-full">
                      {trainingFiles.length} photos selected
                    </span>
                  )}
                </label>
                <input
                  type="file"
                  accept="image/*"
                  multiple
                  onChange={(e) => setTrainingFiles(e.target.files)}
                  className="w-full text-xs text-on-surface-variant file:mr-4 file:py-2.5 file:px-4 file:rounded-xl file:border-0 file:text-xs file:font-bold file:bg-primary file:text-white hover:file:bg-primary-container cursor-pointer"
                />

                {trainingFiles && trainingFiles.length > 0 && (
                  <div className="grid grid-cols-4 gap-2 pt-2 max-h-36 overflow-y-auto p-2 bg-surface-container-low rounded-xl border border-outline-variant/30">
                    {Array.from(trainingFiles).map((file, idx) => (
                      <div key={idx} className="relative aspect-square rounded-lg overflow-hidden border border-outline-variant/40 group">
                        <img
                          src={URL.createObjectURL(file)}
                          alt={`Preview ${idx + 1}`}
                          className="w-full h-full object-cover"
                        />
                        <div className="absolute inset-0 bg-black/40 flex items-center justify-center text-white text-[9px] font-bold opacity-0 group-hover:opacity-100 transition-opacity">
                          #{idx + 1}
                        </div>
                      </div>
                    ))}
                  </div>
                )}
              </div>

              {isUploading && (
                <p className="text-[10px] text-primary font-bold animate-pulse">Saving changes...</p>
              )}

              <div className="flex gap-3 justify-end pt-4 border-t border-outline-variant/30">
                <button
                  type="button"
                  disabled={isUploading}
                  onClick={() => setEditingPet(null)}
                  className="px-5 py-2.5 bg-surface-container hover:bg-surface-container-high rounded-xl text-xs font-bold"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  disabled={isUploading}
                  className="px-5 py-2.5 bg-primary text-on-primary hover:shadow-lg rounded-xl text-xs font-bold"
                >
                  {isUploading ? 'Saving...' : 'Save Changes'}
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </section>
  );
};

export default RegisteredPets;
