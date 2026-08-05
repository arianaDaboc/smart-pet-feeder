export default function Home() {
  return (
    <main style={{ fontFamily: 'system-ui', maxWidth: 720, margin: '60px auto' }}>
      <h1>Guardian Local AI Server</h1>
      <p>Serverul CLIP rulează local. Verifică <a href="/api/status">/api/status</a>.</p>
    </main>
  );
}
