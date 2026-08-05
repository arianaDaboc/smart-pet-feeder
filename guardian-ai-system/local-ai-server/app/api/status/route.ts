import { NextResponse } from 'next/server';
import { corsHeaders, optionsResponse } from '@/lib/cors';
import { loadDatabase } from '@/lib/embeddings';

export const runtime = 'nodejs';
export const dynamic = 'force-dynamic';
export function OPTIONS() { return optionsResponse(); }

export async function GET() {
  const db = loadDatabase();
  return NextResponse.json({
    online: true,
    model: 'Xenova/clip-vit-base-patch32',
    enrolledPets: Object.entries(db).map(([petId, pet]) => ({ petId, name: pet.name, samples: pet.embeddings.length })),
  }, { headers: corsHeaders });
}
