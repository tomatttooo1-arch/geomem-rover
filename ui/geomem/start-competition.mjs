import {createAppServer} from './app/server.mjs';
import {spawn} from 'node:child_process';
let server;
for(let port=8791;port<8810;port++){
  const candidate=createAppServer();
  try{await new Promise((resolve,reject)=>{candidate.once('error',reject);candidate.listen(port,'127.0.0.1',resolve);});server=candidate;break;}
  catch(e){if(!['EADDRINUSE','EACCES'].includes(e.code))throw e;}
}
if(!server)throw Error('No local port available');
const url=`http://127.0.0.1:${server.address().port}/competition.html?sim=1`;
console.log('GeoMem competition rehearsal: '+url+'\nSimulation only. Keep this window open. Ctrl+C stops the local server.');
if(process.platform==='win32'){const p=spawn('rundll32.exe',['url.dll,FileProtocolHandler',url],{windowsHide:true,stdio:'ignore'});p.on('error',()=>{});p.unref();}
