// 自我注销的 Service Worker：旧版（geomem-m8-v*）会在服务停掉后继续用缓存显示旧演示页。
// 页面已不再注册 SW；已装过旧版的浏览器在下次检查更新时拿到这一份，清掉缓存并注销自己。
self.addEventListener('install',()=>self.skipWaiting());
self.addEventListener('activate',e=>e.waitUntil(caches.keys().then(keys=>Promise.all(keys.filter(k=>k.startsWith('geomem-m8-')).map(k=>caches.delete(k)))).then(()=>self.registration.unregister())));
