import { NextRequest, NextResponse } from 'next/server';
import { corsHeaders, optionsResponse } from '@/lib/cors';
import { getEmbedding, updateDatabase } from '@/lib/embeddings';

export const runtime = 'nodejs';
export const dynamic = 'force-dynamic';
export function OPTIONS() { return optionsResponse(); }

export async function POST(request: NextRequest) {
  try {
    const form = await request.formData();
    const file = form.get('image');
    const petId = String(form.get('petId') || '').trim();
    const petName = String(form.get('petName') || '').trim();
    if (!(file instanceof File) || !petId || !petName) {
      return NextResponse.json({ error: 'image, petId and petName are required' }, { status: 400, headers: corsHeaders });
    }
    const embedding = await getEmbedding(Buffer.from(await file.arrayBuffer()));
    await updateDatabase(db => {
      const pet = db[petId] || { name: petName, embeddings: [] };
      pet.name = petName;
      if (pet.embeddings.length < 40) pet.embeddings.push(embedding);
      db[petId] = pet;
    });
    return NextResponse.json({ enrolled: true, petId, petName }, { headers: corsHeaders });
  } catch (error) {
    return NextResponse.json({ error: error instanceof Error ? error.message : 'Enrollment failed' }, { status: 500, headers: corsHeaders });
  }
}
