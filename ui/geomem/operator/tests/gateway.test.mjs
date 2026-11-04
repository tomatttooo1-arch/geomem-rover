import test from 'node:test';
import assert from 'node:assert/strict';
import http from 'node:http';
import net from 'node:net';
import {once} from 'node:events';
import {createAppServer} from '../server.mjs';

test('same-origin gateway forwards bridge bytes, strips camera credentials, and rejects legacy driving', async () => {
  const paths=[];
  const sockets=new Set();
  const upstream=http.createServer((req,res)=>{
    paths.push(req.url);
    if(req.url==='/status')res.setHeader('Content-Type','application/json');
    res.end(req.url==='/status'?JSON.stringify({source:'live',camera:true,frameAgeMs:5,motionEnabled:false,teleopSocket:false,ticket:'private',controller:'private'}):'asset');
  });
  upstream.on('connection',socket=>{sockets.add(socket);socket.on('close',()=>sockets.delete(socket));});
  let upgradeHeaders;
  upstream.on('upgrade',(req,socket,head)=>{
    upgradeHeaders=req.headers;
    assert.equal(req.url,'/ws');
    socket.write('HTTP/1.1 101 Switching Protocols\r\nConnection: Upgrade\r\nUpgrade: websocket\r\n\r\n');
    if(head.length)socket.write(head);
    socket.on('data',data=>socket.write(data));
    socket.on('error',()=>{});
  });
  upstream.listen(0,'127.0.0.1');await once(upstream,'listening');
  const upPort=upstream.address().port;
  process.env.GEOMEM_BRIDGE_PORT=String(upPort);process.env.ROVER_PORT=String(upPort);process.env.GEOMEM_MAPPING_PORT=String(upPort);
  const app=createAppServer();app.listen(0,'127.0.0.1');await once(app,'listening');
  app.on('connection',socket=>{sockets.add(socket);socket.on('close',()=>sockets.delete(socket));});
  const port=app.address().port,base=`http://127.0.0.1:${port}`;
  try {
    assert.equal(await(await fetch(base+'/bridge/preview.jpg')).text(),'asset');
    assert.equal(await(await fetch(base+'/mapping/cloud.bin?t=1')).text(),'asset');
    assert.equal((await fetch(base+'/mapping/token.txt')).status,404);
    assert.equal((await fetch(base+'/mapping/cloud.bin?t=2',{method:'POST'})).status,404);
    assert.equal((await fetch(base+'/mapping/cloud.bin?t=3',{headers:{Origin:'https://foreign.example'}})).status,403);
    const sha='a'.repeat(64);
    assert.equal((await fetch(base+`/bridge/basemap/${sha}/cells.bin`)).status,200);
    assert.equal((await fetch(base+'/bridge/token.txt')).status,404);
    assert.equal((await fetch(base+'/bridge/preview.jpg',{headers:{Origin:'https://foreign.example'}})).status,403);
    assert.equal((await fetch(base+'/rover/drive',{method:'POST',body:'{"v":1}'})).status,405);
    const status=await(await fetch(base+'/rover/status')).json();
    assert.equal(status.source,'live');assert.equal(status.ticket,undefined);assert.equal(status.controller,undefined);
    const exchange=async(origin,path='/bridge/ws')=>{
      const socket=net.connect(port,'127.0.0.1');await once(socket,'connect');
      socket.setTimeout(3000,()=>socket.destroy(new Error('gateway timeout')));
      socket.write(`GET ${path} HTTP/1.1\r\nHost: 127.0.0.1:${port}\r\nOrigin: ${origin}\r\nConnection: Upgrade\r\nUpgrade: websocket\r\n\r\npayload`);
      let result='';
      try {for await(const chunk of socket){result+=chunk.toString();if(result.includes('payload')||result.startsWith('HTTP/1.1 403'))break;}}finally{socket.destroy();}
      return result;
    };
    assert.match(await exchange(base),/^HTTP\/1.1 101/);
    assert.equal(upgradeHeaders.origin,`http://127.0.0.1:${upPort}`);
    assert.equal(upgradeHeaders.host,`127.0.0.1:${upPort}`);
    assert.match(await exchange('https://foreign.example'),/^HTTP\/1.1 403/);
    assert.match(await exchange(base,'/other'),/^HTTP\/1.1 403/);
    assert(!paths.includes('/drive'));
  } finally {
    for(const socket of sockets)socket.destroy();
    app.closeAllConnections();app.close();upstream.closeAllConnections();upstream.close();
    delete process.env.GEOMEM_BRIDGE_PORT;delete process.env.ROVER_PORT;delete process.env.GEOMEM_MAPPING_PORT;
  }
});
