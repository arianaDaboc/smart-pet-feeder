import { NextRequest, NextResponse } from 'next/server';
import { corsHeaders, optionsResponse } from '@/lib/cors';
import { findBestMatch, getEmbedding } from '@/lib/embeddings';

export const runtime = 'nodejs';
export const dynamic = 'force-dynamic';
export function OPTIONS() { return optionsResponse(); }

export async function POST(request: NextRequest) {
  try {
    const form = await request.formData();
    const file = form.get('image');
    const thresholdValue = form.get('threshold');
    const requestedThreshold = typeof thresholdValue === 'string' ? Number(thresholdValue) : undefined;
    if (!(file instanceof File)) {
      return NextResponse.json({ error: 'image is required' }, { status: 400, headers: corsHeaders });
    }
    const embedding = await getEmbedding(Buffer.from(await file.arrayBuffer()));
    return NextResponse.json(findBestMatch(embedding, requestedThreshold), { headers: corsHeaders });
  } catch (error) {
    return NextResponse.json({ error: error instanceof Error ? error.message : 'Recognition failed' }, { status: 500, headers: corsHeaders });
  }
}
