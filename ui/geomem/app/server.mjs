import http from 'node:http';
import { readFile, realpath } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { realpathSync } from 'node:fs';
const root = realpathSync(path.dirname(fileURLToPath(import.meta.url)));
const mime = { '.html':'text/html; charset=utf-8', '.css':'text/css', '.js':'text/javascript', '.json':'application/json', '.webmanifest':'application/manifest+json', '.svg':'image/svg+xml', '.png':'image/png' };
export function createAppServer() {
return http.createServer(async (req, res) => {
  try {
    if (!['GET', 'HEAD'].includes(req.method)) { res.writeHead(405).end(); return; }
    const url = new URL(req.url, 'http://localhost');
    const requested = decodeURIComponent(url.pathname).replace(/^\/+/, '') || 'index.html';
    // 先解析成真实路径，再按相对 root 的首段判白名单（防 src/../server.mjs 这类绕过）。
    const file = await realpath(path.resolve(root, requested));
    const relative = path.relative(root, file);
    const first = relative.split(path.sep)[0];
    const allowedTop = ['index.html','styles.css','manifest.webmanifest','sw.js','kiosk.html','kiosk.css','competition.html'];
    if (!relative || relative.startsWith('..') || path.isAbsolute(relative) || !(allowedTop.includes(relative) || (['src','vendor','assets'].includes(first) && relative.includes(path.sep)))) { res.writeHead(404).end(); return; }
    const bytes = await readFile(file);
    res.writeHead(200, {'Content-Type':mime[path.extname(file)] || 'application/octet-stream', 'Cache-Control':'no-cache', 'X-Content-Type-Options':'nosniff', 'Referrer-Policy':'no-referrer'});
    res.end(req.method === 'HEAD' ? undefined : bytes);
  } catch { res.writeHead(404).end('Not found'); }
});
}
if (process.argv[1] && import.meta.url === pathToFileURL(path.resolve(process.argv[1])).href) {
  const port = Number(process.env.PORT || 8788);
  const server = createAppServer();
  server.listen(port, process.env.HOST || '127.0.0.1', () => console.log(`GeoMem M8: http://127.0.0.1:${server.address().port}`));
}
