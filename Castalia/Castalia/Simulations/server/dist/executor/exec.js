'use strict';

Object.defineProperty(exports, "__esModule", {
  value: true
});
exports.register = register;

var _fs = require('fs');

var _fs2 = _interopRequireDefault(_fs);

var _cmdPromise = require('cmd-promise');

var _cmdPromise2 = _interopRequireDefault(_cmdPromise);

var _bluebird = require('bluebird');

var _bluebird2 = _interopRequireDefault(_bluebird);

var _session = require('../session');

var _session2 = _interopRequireDefault(_session);

function _interopRequireDefault(obj) { return obj && obj.__esModule ? obj : { default: obj }; }

const randomSessionId = length => {
  const possible = "ABCDEFGHIJ" + "KLMNOPQRSTUVWXYZabcdef" + "ghijklmnopqrstuvwxyz0123456789";

  let text = "";
  for (let i = 0; i < length; i++) text += possible.charAt(Math.floor(Math.random() * possible.length));

  return text;
};

let queue = [];

function register(config, callback) {
  let sessionId = randomSessionId(5);
  queue.push({ sessionId, config });
  _session2.default.markStatus(sessionId, "enqueued", "Simulation is registered, waiting to be executed");
  callback(null, {
    status: "OK",
    sessionId: sessionId,
    error: null
  });

  if (queue.length === 1) {
    const { config, sessionId } = queue.shift();
    exec({ config, sessionId });
  }
}

function exec({ config, sessionId }) {
  const {
    mode = "debug",
    network: {
      fieldWidth, fieldHeight,
      nodes, nodeConfig: { routingAlgorithm, macInterface }
    },
    simulation: {
      traffic = [], timeLimit
    }
  } = config;
  console.log(traffic);
  (0, _cmdPromise2.default)(`rm -f Castalia-Trace.txt`);

  let fileName = `${sessionId}.ini`;
  const requestFileName = `logs/${sessionId}_request.txt`;
  _fs2.default.writeFile(requestFileName, JSON.stringify(config), () => {});
  console.log(traffic);

  let nextId = 0;
  let o2n = {},
      n2o = {};

  (0, _cmdPromise2.default)(`
    copy template\\template.ini ${fileName}
  `).then(() => {
    let writer = _fs2.default.createWriteStream(fileName, { 'flags': 'a' });

    if (routingAlgorithm) {
      if (routingAlgorithm.toLowerCase() === 'greedy') {
        writer.write(`SN.node[*].Communication.RoutingProtocolName = "GreedyRouting"\n`);
      } else if (routingAlgorithm.toLowerCase() === 'gpsr') {
        writer.write(`SN.node[*].Communication.RoutingProtocolName = "GpsrRouting"\n`);
      } else if (routingAlgorithm.toLowerCase() === 'rollingball') {
        writer.write(`SN.node[*].Communication.RoutingProtocolName = "RollingBallRouting"\n`);
      } else if (routingAlgorithm.toLowerCase() === 'stable') {
        writer.write(`SN.node[*].Communication.RoutingProtocolName = "StableRouting"\n`);
      } else if (routingAlgorithm.toLowerCase() === 'shortestpath') {
        writer.write(`SN.node[*].Communication.RoutingProtocolName = "ShortestPathRouting"\n`);
      } else if (routingAlgorithm.toLowerCase() === 'mlp') {
        writer.write(`SN.node[*].Communication.RoutingProtocolName = "MlpRouting"\n`);
      } else {
        writer.write(`SN.node[*].Communication.RoutingProtocolName = "GpsrRouting"\n`);
      }
    } else {
      writer.write(`SN.node[*].Communication.RoutingProtocolName = "GpsrRouting"\n`);
    }

    writer.write(`SN.numNodes = ${nodes.length}\n`);
    writer.write(`SN.field_x = ${fieldWidth}\n`);
    writer.write(`SN.field_y = ${fieldHeight}\n`);

    for (let node of nodes) {
      const { id, x, y } = node;
      writer.write(`SN.node[${id}].xCoor = ${x}\n`);
      writer.write(`SN.node[${id}].yCoor = ${y}\n`);
    }

    writer.write(`sim-time-limit = 30s\n`);

    for (let i = 0; i < traffic.length; i++) {
      let pair = traffic[i];
      const { source, destination } = pair;
      writer.write(`SN.node[${source}].Application.isSource = true\n`);
      writer.write(`SN.node[${source}].Application.sink = "${destination}"\n`);
      // writer.write(`SN.node[${source}].Application.startSendingTime = ${5 + i * 10}\n`);
      // writer.write(`SN.node[${source}].Application.stopSendingTime = ${5 + i * 10 + 10}\n`);
    }

    // sim-time-limit = 25s


    if (mode === "debug") {
      writer.write(`SN.node[*].Application.numPacketToSend = 10\n`);
      writer.write(`SN.isDebugMode = true\n`);
    } else {
      writer.write(`SN.isDebugMode = false\n`);
    }

    writer.end();
    _session2.default.markStatus(sessionId, "running", "Simulation is running...");

    _fs2.default.writeFile('D:\\SEDIC\\WSN\\posse\\Castalia\\Castalia\\CastaliaBin3.sh', `Castalia -i ${fileName} -c General`, function (err) {
      if (err) throw err;
    });
    return (0, _cmdPromise2.default)(`
      start mingwenv3.cmd 
    `); //D:\\SEDIC\\WSN\\posse\\Castalia\\Castalia\\
  }).then(() => {
    (0, _cmdPromise2.default)(`move ${fileName} archives`);

    const eventTraceFileName = `logs/${sessionId}_eventTrace.txt`;
    const energyTraceFileName = `logs/${sessionId}_energyTrace.txt`;
    const nodeTraceFileName = `logs/${sessionId}_nodeTrace.txt`;
    const drawFileName = `logs/${sessionId}_draw.txt`;
    const logFileName = `logs/${sessionId}_log.txt`;
    const statisticsFileName = `logs/${sessionId}_statistics.txt`;
    let eventWriter = _fs2.default.createWriteStream(eventTraceFileName);
    let energyWriter = _fs2.default.createWriteStream(energyTraceFileName);
    let nodeWriter = _fs2.default.createWriteStream(nodeTraceFileName);
    let drawWriter = _fs2.default.createWriteStream(drawFileName);
    let logWriter = _fs2.default.createWriteStream(logFileName);
    let statisticsWriter = _fs2.default.createWriteStream(statisticsFileName);

    let statistics = [];

    let lines = _fs2.default.readFileSync('Castalia-Trace.txt').toString().split('\n');
    for (let line of lines) {
      if (/WSN_EVENT/gi.test(line)) {
        const time = getTime(line);
        const type = getType(line);

        if (type === 'ENERGY') {
          let regex = /id:(\d+) energy:([\d\.]+)/gi;
          let match = regex.exec(line);
          if (match) {
            let id = parseInt(match[1]);
            let energy = parseFloat(match[2]);
            let event = { "e": "E", "id": id, "time": time, "energy": energy };
            energyWriter.write(`${JSON.stringify(event)}\n`);
          }
        } else if (type === 'FINAL') {
          let regex = /id:(\d+) x:([\d\.]+) y:([\d\.]+)/gi;
          let match = regex.exec(line);
          if (match) {
            let id = parseInt(match[1]);
            let x = parseFloat(match[2]);
            let y = parseFloat(match[3]);
            let node = { "id": id, "x": x, "y": y, "d_time": -1 };
            nodeWriter.write(`${JSON.stringify(node)}\n`);
          }
        } else if (type === 'SEND' || type === 'FORWARD' || type === 'RECEIVE' || type === 'DROP') {
          let regex = /packetId:(\d+) source:(\d+) destination:(\d+) current:(\d+)/;
          let match = regex.exec(line);
          if (match) {
            let packetId = parseInt(match[1]);
            let source = parseInt(match[2]);
            let destination = parseInt(match[3]);
            let current = parseInt(match[4]);
            let event = { "e": type[0], "p_id": packetId, "p_type": "normal",
              "source": source, "dest": destination, "cur": current, "layer": "network", "time": time };
            eventWriter.write(`${JSON.stringify(event)}\n`);
          }
        } else if (type === 'DRAW') {
          // console.log(line);
          let lineRegex = /LINE ([-\d\.]+) ([-\d\.]+) ([-\d\.]+) ([-\d\.]+) (\S+)/;
          let lineMatch = lineRegex.exec(line);
          if (lineMatch) {
            let x1 = parseFloat(lineMatch[1]);
            let y1 = parseFloat(lineMatch[2]);
            let x2 = parseFloat(lineMatch[3]);
            let y2 = parseFloat(lineMatch[4]);
            let color = lineMatch[5];
            drawWriter.write(`${JSON.stringify({ type: 'line', x1, y1, x2, y2, color })}\n`);
          }

          let circleRegex = /CIRCLE ([-\d\.]+) ([-\d\.]+) ([\d\.]+) (\S+)/;
          let circleMatch = circleRegex.exec(line);
          if (circleMatch) {
            let centerX = parseFloat(circleMatch[1]);
            let centerY = parseFloat(circleMatch[2]);
            let radius = parseFloat(circleMatch[3]);
            let color = circleMatch[4];
            drawWriter.write(`${JSON.stringify({ type: 'circle', centerX, centerY, radius, color })}\n`);
          }

          let pointRegex = /POINT ([-\d\.]+) ([-\d\.]+) (\S+)/;
          let pointMatch = pointRegex.exec(line);
          if (pointMatch) {
            let x = parseFloat(pointMatch[1]);
            let y = parseFloat(pointMatch[2]);
            let color = pointMatch[3];
            drawWriter.write(`${JSON.stringify({ type: 'point', x, y, color })}\n`);
          }

          let arcRegex = /ARC ([-\d\.]+) ([-\d\.]+) ([-\d\.]+) ([-\d\.]+) ([\d\.]+) (\S+)/;
          let arcMatch = arcRegex.exec(line);
          if (arcMatch) {
            let fromX = parseFloat(arcMatch[1]);
            let fromY = parseFloat(arcMatch[2]);
            let toX = parseFloat(arcMatch[3]);
            let toY = parseFloat(arcMatch[4]);
            let radius = parseFloat(arcMatch[5]);
            let color = arcMatch[6];
            drawWriter.write(`${JSON.stringify({ type: 'arc', fromX, fromY, toX, toY, radius, color })}\n`);
          }
        } else if (type === 'STATISTICS') {
          let regex = /id:(\d+) totalPacketReceived:(\d+) estimateLifetime:([\d\.]+) x:([\d\.]+) y:([\d\.]+) sumHopRatio:([\d\.]+) endPointCount:([\d\.]+) energyConsumed:([\d\.]+)/gi;
          let match = regex.exec(line);

          if (match) {
            let id = parseInt(match[1]);
            let totalPacketReceived = parseInt(match[2]);
            let estimateLifetime = parseFloat(match[3]);
            let x = parseFloat(match[4]);
            let y = parseFloat(match[5]);
            let sumHopRatio = parseFloat(match[6]);
            let endPointCount = parseFloat(match[7]);
            let energyConsumed = parseFloat(match[8]);
            statistics[id] = { id, totalPacketReceived,
              estimateLifetime, x, y, sumHopRatio, endPointCount, energyConsumed };
          }
        }
      } else if (/WSN_LOG/gi.test(line)) {
        let regex = /WSN_LOG (.+)/;
        let match = regex.exec(line);
        if (match) {
          logWriter.write(`${match[1]}\n`);
        }
      }
    }

    statisticsWriter.write(JSON.stringify(statistics));

    energyWriter.end();
    eventWriter.end();
    nodeWriter.end();
    drawWriter.end();
    logWriter.end();
    statisticsWriter.end();
    _session2.default.markStatus(sessionId, "completed", "Simulation completed");
    if (queue.length >= 1) {
      const { config, sessionId } = queue.shift();
      exec({ config, sessionId });
    }
  }).catch(err => {
    _session2.default.markStatus(sessionId, "error", err.toString());
    if (queue.length >= 1) {
      const { config, sessionId } = queue.shift();
      exec({ config, sessionId });
    }
  });
}

const getType = line => {
  let regex = /WSN_EVENT (\S+)/gi;
  let match = regex.exec(line);

  if (match) {
    return match[1];
  }
  return null;
};

const getTime = line => {
  let regex = /^(\d+\.\d+)/gi;
  let match = regex.exec(line);

  if (match) {
    return parseFloat(match[1]);
  }

  return 0;
};
//# sourceMappingURL=exec.js.map