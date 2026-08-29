import { mutation, query } from "./_generated/server";
import { v } from "convex/values";

export const list = query({
  args: { ownerId: v.string() },
  handler: async (ctx, args) => {
    const allPets = await ctx.db
      .query("pets")
      .withIndex("by_ownerId", (q) => q.eq("ownerId", args.ownerId))
      .collect();

    const activePets = allPets.filter((p) => p.isActive === true);

    const resolvedPets = [];
    for (const pet of activePets) {
      let resolvedProfileImage = pet.profileImage;
      if (pet.profileImage && !pet.profileImage.startsWith("http")) {
        resolvedProfileImage = (await ctx.storage.getUrl(pet.profileImage)) || undefined;
      }

      const resolvedTrainingImages = [];
      if (pet.trainingImages) {
        for (const img of pet.trainingImages) {
          if (img && !img.startsWith("http")) {
            const url = await ctx.storage.getUrl(img);
            if (url) resolvedTrainingImages.push(url);
          } else {
            resolvedTrainingImages.push(img);
          }
        }
      }

      resolvedPets.push({
        ...pet,
        profileImage: resolvedProfileImage,
        trainingImages: resolvedTrainingImages,
      });
    }

    return resolvedPets;
  },
});

export const add = mutation({
  args: {
    ownerId: v.string(),
    name: v.string(),
    species: v.string(),
    breed: v.string(),
    age: v.string(),
    weightKg: v.optional(v.float64()),
    dailyGoalGrams: v.optional(v.float64()),
    lifeStage: v.optional(v.string()),
    neutered: v.optional(v.boolean()),
    activityLevel: v.optional(v.string()),
    bodyConditionScore: v.optional(v.float64()),
    targetWeightKg: v.optional(v.float64()),
    foodKcalPer100g: v.optional(v.float64()),
    foodType: v.optional(v.string()),
    allergies: v.optional(v.string()),
    medicalConditions: v.optional(v.string()),
    profileImage: v.optional(v.string()),
    trainingImages: v.array(v.string()),
    aiModelStatus: v.string(),
  },
  handler: async (ctx, args) => {
    return await ctx.db.insert("pets", {
      ownerId: args.ownerId,
      name: args.name,
      species: args.species,
      breed: args.breed,
      age: args.age,
      weightKg: args.weightKg,
      dailyGoalGrams: args.dailyGoalGrams,
      lifeStage: args.lifeStage,
      neutered: args.neutered,
      activityLevel: args.activityLevel,
      bodyConditionScore: args.bodyConditionScore,
      targetWeightKg: args.targetWeightKg,
      foodKcalPer100g: args.foodKcalPer100g,
      foodType: args.foodType,
      allergies: args.allergies,
      medicalConditions: args.medicalConditions,
      profileImage: args.profileImage,
      trainingImages: args.trainingImages,
      aiModelStatus: args.aiModelStatus,
      isActive: true,
      createdAt: Date.now(),
    });
  },
});

export const remove = mutation({
  args: { id: v.id("pets") },
  handler: async (ctx, args) => {

    await ctx.db.patch(args.id, {
      isActive: false,
    });
  },
});

export const update = mutation({
  args: {
    id: v.id("pets"),
    name: v.optional(v.string()),
    species: v.optional(v.string()),
    breed: v.optional(v.string()),
    age: v.optional(v.string()),
    weightKg: v.optional(v.float64()),
    dailyGoalGrams: v.optional(v.float64()),
    lifeStage: v.optional(v.string()),
    neutered: v.optional(v.boolean()),
    activityLevel: v.optional(v.string()),
    bodyConditionScore: v.optional(v.float64()),
    targetWeightKg: v.optional(v.float64()),
    foodKcalPer100g: v.optional(v.float64()),
    foodType: v.optional(v.string()),
    allergies: v.optional(v.string()),
    medicalConditions: v.optional(v.string()),
    profileImage: v.optional(v.string()),
    trainingImages: v.optional(v.array(v.string())),
    aiModelStatus: v.optional(v.string()),
  },
  handler: async (ctx, args) => {
    const patchData: any = {};
    if (args.name !== undefined) patchData.name = args.name;
    if (args.species !== undefined) patchData.species = args.species;
    if (args.breed !== undefined) patchData.breed = args.breed;
    if (args.age !== undefined) patchData.age = args.age;
    if (args.weightKg !== undefined) patchData.weightKg = args.weightKg;
    if (args.dailyGoalGrams !== undefined) patchData.dailyGoalGrams = args.dailyGoalGrams;
    if (args.lifeStage !== undefined) patchData.lifeStage = args.lifeStage;
    if (args.neutered !== undefined) patchData.neutered = args.neutered;
    if (args.activityLevel !== undefined) patchData.activityLevel = args.activityLevel;
    if (args.bodyConditionScore !== undefined) patchData.bodyConditionScore = args.bodyConditionScore;
    if (args.targetWeightKg !== undefined) patchData.targetWeightKg = args.targetWeightKg;
    if (args.foodKcalPer100g !== undefined) patchData.foodKcalPer100g = args.foodKcalPer100g;
    if (args.foodType !== undefined) patchData.foodType = args.foodType;
    if (args.allergies !== undefined) patchData.allergies = args.allergies;
    if (args.medicalConditions !== undefined) patchData.medicalConditions = args.medicalConditions;
    if (args.profileImage !== undefined) patchData.profileImage = args.profileImage;
    if (args.trainingImages !== undefined) patchData.trainingImages = args.trainingImages;
    if (args.aiModelStatus !== undefined) patchData.aiModelStatus = args.aiModelStatus;

    await ctx.db.patch(args.id, patchData);
  },
});
