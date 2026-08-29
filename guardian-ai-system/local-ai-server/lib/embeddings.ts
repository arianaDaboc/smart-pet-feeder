import { pipeline, RawImage } from '@huggingface/transformers';
import fs from 'node:fs';
import path from 'node:path';

export interface PetDatabase {
  [petId: string]: { name: string; embeddings: number[][] };
}

const DB_PATH = path.join(process.cwd(), 'data', 'pets.json');
let extractorPromise: Promise<any> | null = null;
let writeQueue = Promise.resolve();

function getExtractor() {
  if (!extractorPromise) {
    extractorPromise = pipeline('image-feature-extraction', 'Xenova/clip-vit-base-patch32', { dtype: 'q8' });
  }
  return extractorPromise;
}

export async function getEmbedding(imageBuffer: Buffer): Promise<number[]> {
  const extractor = await getExtractor();
  const arrayBuffer = new ArrayBuffer(imageBuffer.byteLength);
  new Uint8Array(arrayBuffer).set(imageBuffer);
  const image = await RawImage.fromBlob(new Blob([arrayBuffer]));
  const output = await extractor(image, { pooling: 'mean', normalize: true });
  const values = Array.from(output.data as Float32Array);
  const norm = Math.sqrt(values.reduce((sum, value) => sum + value * value, 0));
  if (!Number.isFinite(norm) || norm <= 0) throw new Error('CLIP returned an invalid embedding.');

  return values.map(value => value / norm);
}

export function loadDatabase(): PetDatabase {
  if (!fs.existsSync(DB_PATH)) return {};
  try {
    return JSON.parse(fs.readFileSync(DB_PATH, 'utf8')) as PetDatabase;
  } catch {
    return {};
  }
}

export async function updateDatabase(mutator: (db: PetDatabase) => void) {
  writeQueue = writeQueue.then(async () => {
    const db = loadDatabase();
    mutator(db);
    fs.mkdirSync(path.dirname(DB_PATH), { recursive: true });
    const temporaryPath = `${DB_PATH}.tmp`;
    fs.writeFileSync(temporaryPath, JSON.stringify(db, null, 2));
    fs.renameSync(temporaryPath, DB_PATH);
  });
  await writeQueue;
}

function dotProduct(a: number[], b: number[]) {
  const length = Math.min(a.length, b.length);
  let sum = 0;
  for (let index = 0; index < length; index += 1) sum += a[index] * b[index];
  return sum;
}

export function findBestMatch(embedding: number[], requestedThreshold?: number) {
  const db = loadDatabase();
  const configuredThreshold = Number(process.env.CLIP_SIMILARITY_THRESHOLD || 0.88);
  const threshold = Number.isFinite(requestedThreshold)
    ? Math.min(0.98, Math.max(0.80, requestedThreshold as number))
    : configuredThreshold;
  const marginRequired = Number(process.env.CLIP_MINIMUM_MARGIN || 0.04);
  const candidates = Object.entries(db).map(([petId, pet]) => {
    const scores = pet.embeddings.map(stored => dotProduct(embedding, stored)).sort((a, b) => b - a);
    const topScores = scores.slice(0, Math.min(3, scores.length));
    const score = topScores.length ? topScores.reduce((sum, value) => sum + value, 0) / topScores.length : -1;
    return { petId, pet: pet.name, similarity: score };
  }).sort((a, b) => b.similarity - a.similarity);

  const best = candidates[0];
  const second = candidates[1];
  const margin = best ? best.similarity - (second?.similarity ?? 0) : 0;
  const authorized = !!best && best.similarity >= threshold && (!second || margin >= marginRequired);
  return {
    authorized,
    petId: authorized ? best.petId : null,
    pet: authorized ? best.pet : null,
    similarity: Number((best?.similarity ?? -1).toFixed(4)),
    margin: Number(margin.toFixed(4)),
    threshold,
    candidates: candidates.slice(0, 3).map(item => ({ ...item, similarity: Number(item.similarity.toFixed(4)) })),
  };
}
