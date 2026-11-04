import { spawn } from 'node:child_process';
import { createAppServer } from './app/server.mjs';

const preferredPort = Number(process.env.GEOMEM_PORT || 8788);
if (!Number.isInteger(preferredPort) || preferredPort < 1024 || preferredPort > 65515) {
  console.error('GEOMEM_PORT must be between 1024 and 65515.');
  process.exit(1);
}

async function listen(port) {
  const server = createAppServer();
  return new Promise((resolve, reject) => {
    server.once('error', reject);
    server.listen(port, '127.0.0.1', () => {
      server.removeListener('error', reject);
      resolve(server);
    });
  });
}

let server;
for (let port = preferredPort; port < preferredPort + 20; port++) {
  try { server = await listen(port); break; }
  catch (error) {
    if (!['EADDRINUSE', 'EACCES'].includes(error.code)) throw error;
  }
}
if (!server) {
  console.error('启动失败：可用端口不足，请关闭其他 GeoMem 启动窗口后重试。');
  process.exit(1);
}

const url = `http://127.0.0.1:${server.address().port}`;
console.log(`GeoMem M8: ${url}`);
console.log('智能寻物小车 · 界面演示版');
console.log('浏览器将自动打开。若未打开，请复制上方地址到 Edge / Chrome。');
console.log('请保留此窗口；关闭窗口或按 Ctrl+C 即可停止服务。');
console.log('当前默认使用模拟数据，不代表实车已连接。');
if (!process.argv.includes('--no-open')) {
  const opener = spawn('rundll32.exe', ['url.dll,FileProtocolHandler', url], { windowsHide: true, stdio: 'ignore' });
  opener.on('error', () => console.log(`请手动打开 ${url}`));
  opener.unref();
}
for (const signal of ['SIGINT', 'SIGTERM']) process.once(signal, () => {
  server.close();
  server.closeAllConnections();
});
