// =============================================================================
// Controlled Descent Simulator
// =============================================================================
// File        : ws_rpc_client_pre.js
// Description : pre-js for the libs/ws WebSocket RPC transport (ws_rpc_client).
//               Spawns the WebSocket bridge worker BEFORE the WASM runtime is
//               declared ready: a worker never starts while the main thread is
//               blocked, so it must be up before the first spin-waiting call
//               to ws_rpc(). addRunDependency keeps the module promise pending
//               until the worker reports ready.
// =============================================================================

Module['preRun'] = Module['preRun'] || [];
Module['preRun'].push(function () {
    var g = globalThis;
    if (g.__cdsWs) return; // already initialised (e.g. module re-created)

    if (typeof SharedArrayBuffer === 'undefined') {
        console.error('[cds-ws] SharedArrayBuffer unavailable: the page must be ' +
            'cross-origin isolated. Serve the frontend with COOP/COEP headers ' +
            '(e.g. python3 tools/serve.py). All ws_rpc calls will fail.');
        return;
    }

    /* Worker: owns the WebSocket, shuttles frames <-> shared buffer.
       Buffer layout (see ws_rpc_client.cpp): i32[0] flag (0 idle, 1 request
       pending, 2 response ready, -1 dead), i32[1] data length, i32[2] response
       correlation id, data area from byte 16. Outgoing frames already carry
       the 4-byte correlation id; incoming frames have it stripped here. */
    var src =
        'var i32=null,u8=null,ws=null,ready=false,pending=[];\n' +
        'onmessage=function(e){\n' +
        '  var d=e.data;\n' +
        '  if(d&&d.sab){\n' +
        '    i32=new Int32Array(d.sab);u8=new Uint8Array(d.sab);\n' +
        '    try{ws=new WebSocket(d.url);}catch(x){Atomics.store(i32,0,-1);postMessage(1);return;}\n' +
        '    ws.binaryType="arraybuffer";\n' +
        '    ws.onopen=function(){ready=true;for(var i=0;i<pending.length;i++)ws.send(pending[i]);pending=[];};\n' +
        '    ws.onmessage=function(ev){\n' +
        '      var m=new Uint8Array(ev.data);\n' +
        '      if(m.length<4)return;\n' + /* malformed: no correlation id */
        '      i32[2]=(m[0]|(m[1]<<8)|(m[2]<<16)|(m[3]<<24))|0;\n' +
        /* guard: a payload larger than the data area would throw RangeError in
           u8.set and wedge the transport; signal oversize (len -1) instead */
        '      var n=m.length-4;\n' +
        '      if(n>u8.length-16){i32[1]=-1;Atomics.store(i32,0,2);return;}\n' +
        '      u8.set(m.subarray(4),16);i32[1]=n;\n' +
        '      Atomics.store(i32,0,2);\n' +
        '    };\n' +
        '    ws.onerror=function(){Atomics.store(i32,0,-1);};\n' +
        '    ws.onclose=function(){Atomics.store(i32,0,-1);};\n' +
        '    postMessage(1);\n' + /* ready: worker is up, socket connecting */
        '    return;\n' +
        '  }\n' +
        '  var len=i32[1];\n' +
        '  var buf=u8.slice(16,16+len);\n' + /* copy: cannot send a SAB view */
        '  if(ready)ws.send(buf);else pending.push(buf);\n' +
        '};\n';

    var url = new URLSearchParams(location.search).get('ws') ||
              g.CDS_WS_URL || 'ws://localhost:9002';
    /* 16-byte header + data area. The data area must be >= the largest wire
       message (ws_proto::WS_MAX_MSG_SIZE, currently 4096): sized here to 2x
       that, matching the original headroom. Keep it ahead of WS_MAX_MSG_SIZE if
       that grows — otherwise oversize responses are dropped by the worker's
       guard (they cannot corrupt memory, but the RPC fails). */
    var sab = new SharedArrayBuffer(16 + 8192);
    var w = new Worker(URL.createObjectURL(
        new Blob([src], { type: 'application/javascript' })));

    /* Hold the module ready promise until the worker has actually started:
       once the main thread awaits it, the worker can spin up. */
    addRunDependency('cds-ws-worker');
    w.onmessage = function () {
        w.onmessage = null;
        removeRunDependency('cds-ws-worker');
    };
    w.postMessage({ sab: sab, url: url });

    g.__cdsWs = { i32: new Int32Array(sab), u8: new Uint8Array(sab), w: w, nextId: 0 };
    console.log('[cds-ws] RPC transport connecting to ' + url);
});
