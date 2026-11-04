import http from 'node:http';
import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
const root = path.dirname(fileURLToPath(import.meta.url));
const mime = { '.html':'text/html; charset=utf-8', '.css':'text/css', '.js':'text/javascript', '.json':'application/json', '.webmanifest':'application/manifest+json', '.svg':'image/svg+xml', '.png':'image/png' };
export function createAppServer() {
const bridgePort=Number(process.env.GEOMEM_BRIDGE_PORT||8790);
const localRequest=req=>/^127\.0\.0\.1:\d+$/.test(req.headers.host||'')&&(!req.headers.origin||req.headers.origin===`http://${req.headers.host}`);
const server=http.createServer(async (req, res) => {
  try {
    if(req.url.startsWith('/mapping/')) {
      const route=new URL(req.url,'http://localhost').pathname.slice(9);
      if(!localRequest(req)){res.writeHead(403).end();return;}
      if(req.method!=='GET'||!['status','cloud.bin'].includes(route)){res.writeHead(404).end();return;}
      try{
        await new Promise((resolve,reject)=>{
          const upstream=http.get({host:'127.0.0.1',port:Number(process.env.GEOMEM_MAPPING_PORT||8792),path:`/${route}`,headers:{Host:'127.0.0.1:47802'}},response=>{
            const chunks=[];let size=0;
            response.on('data',chunk=>{size+=chunk.length;if(size>45010000){upstream.destroy(new Error('mapping too large'));return;}chunks.push(chunk);});
            response.on('end',()=>{const body=Buffer.concat(chunks);res.writeHead(response.statusCode,{'Content-Type':response.headers['content-type']||'application/octet-stream','Content-Length':body.length,'Cache-Control':'no-store','X-Content-Type-Options':'nosniff'});res.end(body);resolve();});response.on('error',reject);
          });
          upstream.setTimeout(8000,()=>upstream.destroy(new Error('mapping timeout')));upstream.on('error',reject);
          res.on('close',()=>upstream.destroy());
        });
      }catch{if(!res.headersSent)res.writeHead(503);res.end();}
      return;
    }
    if(req.url.startsWith('/bridge/')) {
      const route=new URL(req.url,'http://localhost').pathname.slice(8);
      if(!localRequest(req)){res.writeHead(403).end();return;}
      if(req.method!=='GET'||!(route==='preview.jpg'||/^basemap\/[a-f0-9]{64}\/(basemap\.json|cells\.bin)$/.test(route))){res.writeHead(404).end();return;}
      try{
        const upstream=await fetch(`http://127.0.0.1:${bridgePort}/${route}`,{signal:AbortSignal.timeout(12000)});
        res.writeHead(upstream.status,{'Content-Type':upstream.headers.get('content-type')||'application/octet-stream','Cache-Control':'no-store','X-Content-Type-Options':'nosniff'});
        for await(const chunk of upstream.body)res.write(chunk);
        res.end();
      }catch{if(!res.headersSent)res.writeHead(503);res.end();}
      return;
    }
    if(req.url.startsWith('/rover/')) {
      const allowed=new Set(['status','frame.jpg']);
      const route=req.url.slice(7).split('?')[0];
      const origin=`http://${req.headers.host}`;
      if(!/^127\.0\.0\.1:\d+$/.test(req.headers.host||'') || (req.headers.origin && req.headers.origin!==origin)) {res.writeHead(403).end();return;}
      if(!allowed.has(route)||req.method!=='GET') {res.writeHead(405).end();return;}
      let body='';
      if(req.method==='POST') {for await(const chunk of req){body+=chunk;if(body.length>1024){res.writeHead(413).end();return;}}}
      try {
        const upstream=await fetch(`http://127.0.0.1:${process.env.ROVER_PORT||18789}/${route}`,{method:req.method,headers:body?{'Content-Type':'application/json'}:undefined,body:body||undefined,signal:AbortSignal.timeout(1500)});
        if(route==='status'){
          const s=await upstream.json();
          res.writeHead(upstream.status,{'Content-Type':'application/json','Cache-Control':'no-store'}).end(JSON.stringify({source:s.source,camera:s.camera,frameAgeMs:s.frameAgeMs,motionEnabled:s.motionEnabled,teleopSocket:s.teleopSocket}));return;
        }
        res.writeHead(upstream.status,{'Content-Type':upstream.headers.get('content-type')||'application/json','Cache-Control':'no-store'});
        res.end(Buffer.from(await upstream.arrayBuffer()));
      } catch {res.writeHead(503,{'Content-Type':'application/json','Cache-Control':'no-store'}).end(JSON.stringify({error:'Nano 通道未连接'}));}
      return;
    }
    if (!['GET', 'HEAD'].includes(req.method)) { res.writeHead(405).end(); return; }
    const url = new URL(req.url, 'http://localhost');
    const relative = decodeURIComponent(url.pathname).replace(/^\/+/, '') || 'index.html';
    const file = path.resolve(root, relative);
    if (!file.startsWith(root + path.sep) || !['index.html','styles.css','manifest.webmanifest','sw.js','src','vendor','assets'].includes(relative.split('/')[0])) { res.writeHead(404).end(); return; }
    const bytes = await readFile(file);
    res.writeHead(200, {'Content-Type':mime[path.extname(file)] || 'application/octet-stream', 'Cache-Control':'no-cache', 'X-Content-Type-Options':'nosniff', 'Referrer-Policy':'no-referrer'});
    res.end(req.method === 'HEAD' ? undefined : bytes);
  } catch { res.writeHead(404).end('Not found'); }
});
server.on('upgrade',(req,socket,head)=>{
  if(req.url!=='/bridge/ws'||!localRequest(req)){socket.end('HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n');return;}
  const upstream=http.request({host:'127.0.0.1',port:bridgePort,path:'/ws',headers:{...req.headers,host:`127.0.0.1:${bridgePort}`,origin:`http://127.0.0.1:${bridgePort}`}});
  socket.on('error',()=>upstream.destroy());
  upstream.on('upgrade',(response,peer,peerHead)=>{
    socket.write(`HTTP/1.1 101 Switching Protocols\r\n${Object.entries(response.headers).map(([k,v])=>`${k}: ${v}`).join('\r\n')}\r\n\r\n`);
    if(peerHead.length)socket.write(peerHead);if(head.length)peer.write(head);
    peer.pipe(socket);socket.pipe(peer);peer.on('error',()=>socket.destroy());socket.on('error',()=>peer.destroy());socket.on('close',()=>peer.destroy());peer.on('close',()=>socket.destroy());
  });
  upstream.on('error',()=>socket.end('HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n'));
  upstream.on('response',response=>{response.resume();socket.end('HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n');});
  socket.on('close',()=>upstream.destroy());upstream.end();
});
return server;
}
if (process.argv[1] && import.meta.url === pathToFileURL(path.resolve(process.argv[1])).href) {
  const port = Number(process.env.PORT || 8788);
  const server = createAppServer();
  server.listen(port, process.env.HOST || '127.0.0.1', () => console.log(`GeoMem M8: http://127.0.0.1:${server.address().port}`));
}
