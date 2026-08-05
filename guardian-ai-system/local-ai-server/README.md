# Guardian Local AI Server

Server Next.js local pentru embeddings CLIP prin pachetul oficial `@huggingface/transformers`. Nu comandă direct servo-ul; returnează doar identitatea, iar aplicația Guardian păstrează logica AI + PIR + cooldown.

```powershell
npm install
npm run dev
```

Prima cerere de enrollment descarcă modelul CLIP și poate dura câteva minute. Serverul trebuie lăsat pornit pe laptop/mini-PC.

- `GET /api/status`
- `POST /api/enroll`: multipart `petId`, `petName`, `image`
- `POST /api/recognize`: multipart `image`

Embeddings persistă în `data/pets.json`. Folosește 15–30 cadre variate per subiect și verifică pragul cu exemple pozitive și negative.
