import { validateSnapshot } from './model.js';
export class LiveAdapter extends EventTarget {
  constructor(url) {super();this.url=url;this.pending=new Map();this.lastSeq=-1;this.ws=null;}
  emit(type,detail) {this.dispatchEvent(new CustomEvent(type,{detail}));}
  connect() {
    this.emit('connection',{state:'connecting'});
    this.ws=new WebSocket(this.url);
    this.timer=setTimeout(()=>{if(this.ws.readyState!==WebSocket.OPEN){this.emit('error','连接超时，请检查桥接服务地址');this.close();}},8000);
    this.ws.onopen=()=>{clearTimeout(this.timer);this.emit('connection',{state:'connected'});};
    this.ws.onmessage=event=>{
      try {
        if(typeof event.data!=='string' || event.data.length>12_000_000) throw Error('消息类型错误或超过 12 MB 上限');
        const data=JSON.parse(event.data);
        if(data.type==='snapshot') {
          validateSnapshot(data);
          if(data.seq<=this.lastSeq) return;
          this.lastSeq=data.seq;this.emit('snapshot',data);
        } else if(data.type==='ack') {
          if(data.version!==1 || typeof data.id!=='string' || typeof data.accepted!=='boolean') throw Error('无效的指令回执');
          const p=this.pending.get(data.id);if(!p)return;
          clearTimeout(p.timer);this.pending.delete(data.id);
          if(data.accepted)p.resolve(data);else p.reject(Error(typeof data.reason==='string'?data.reason:'桥接服务拒绝了请求'));
        }
      } catch(e){this.emit('error',`数据未被采用：${e.message}`);}
    };
    this.ws.onerror=()=>this.emit('error','连接失败，请检查网络、地址和桥接服务。');
    this.ws.onclose=()=>{clearTimeout(this.timer);this.rejectPending();this.emit('connection',{state:'disconnected'});};
  }
  command(action,payload={}) {
    if(this.ws?.readyState!==WebSocket.OPEN)return Promise.reject(Error('连接已断开，请求未发送'));
    const id=crypto.randomUUID();
    return new Promise((resolve,reject)=>{
      const timer=setTimeout(()=>{this.pending.delete(id);reject(Error('回执超时：执行结果未知，请核对小车状态；未自动重发。'));},5000);
      this.pending.set(id,{resolve,reject,timer});
      try {this.ws.send(JSON.stringify({type:'command',version:1,id,action,payload,timestamp:new Date().toISOString()}));}
      catch(e){clearTimeout(timer);this.pending.delete(id);reject(e);}
    });
  }
  rejectPending() {for(const p of this.pending.values()){clearTimeout(p.timer);p.reject(Error('连接中断：请求是否执行未知，请核对小车状态。'));}this.pending.clear();}
  close(){clearTimeout(this.timer);this.rejectPending();if(this.ws){this.ws.onclose=null;this.ws.close();}this.emit('connection',{state:'disconnected'});}
}
