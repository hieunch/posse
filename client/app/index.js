import Network from "./Network";

require('svg.draggable.js');
require('svg.panzoom.js');
import SVG from 'svg.js';
import Mousetrap from 'mousetrap';
import inPolygon from 'point-in-polygon';
import convexHull from 'convex-hull';
import FileSaver from 'file-saver';
import SVGtoPNG from 'save-svg-as-png';
import distanceToLineSegment from 'distance-to-line-segment';
import simpleheat from 'simpleheat';
import data from './data'
import data1 from './data1'
import vis from 'vis';


let printDetail = (data) => {
  let stat = {};
  data.forEach(({totalPacketReceived}) => {
    if (!stat[totalPacketReceived]) {
      stat[totalPacketReceived] = 1;
    } else {
      stat[totalPacketReceived]++;
    }
  });
  console.log(stat);
};

let printStat = (data) => {

  let stat = {};
  data.forEach(({endPointCount}) => {
    if (!stat[endPointCount]) {
      stat[endPointCount] = 1;
    } else {
      stat[endPointCount]++;
    }
  });

  console.log(stat)
};

let handleStatistics = (datas, width, height) => {
  $('#option-panel').show();
  $('#show-result').off('change');
  let on = false;
  $('#show-result').change(() => {
    on = !on;
    if (on) {
      $('#result').show();
      $('#graph-container').hide();
    } else {
      $('#result').hide();
      $('#graph-container').show();
    }
  });

  let max = datas.reduce((acc, val) => {
    return Math.max(acc,
      val.data.reduce((acc, node) => Math.max(acc, node.totalPacketReceived), 0));
  }, 0);
  datas.forEach((result, i) => {
    let {routingAlgorithm, data, numTraffic} = result;
    $('#result').append(`<div id="result${i}"></div>`);
    let outterDiv = $(`#result${i}`);
    outterDiv.append(`<p>${routingAlgorithm}</p>`);
    outterDiv.append(`<canvas id="canvas${i}">${routingAlgorithm}</canvas>`);
    $(`#canvas${i}`).attr('width', width + 100).attr('height', height + 100);
    let heat = simpleheat(`canvas${i}`);
    heat.gradient({
      0.4: 'blue',
      0.6: 'cyan',
      0.7: 'lime',
      0.8: 'yellow',
      1.0: 'red'
    });

    let thresholdMax = 450;
    let thresholdDraw = 80;
    let additive = thresholdMax / 5;
    heat.max(thresholdMax);
    heat.data(data
    // .filter(({totalPacketReceived}) => totalPacketReceived > 0)
      .map(({x, y, totalPacketReceived, energyConsumed}) => {
        let res = 0;
        if (totalPacketReceived > thresholdDraw) res = Math.min(thresholdMax, totalPacketReceived);
        else if (totalPacketReceived > 5) res = totalPacketReceived + additive;
        else res = 0;
        return [x + 50, y + 50, res]
      }));
    heat.draw();
    outterDiv.append(`<div id="surface${i}">${routingAlgorithm}</div>`);
    $(`#surface${i}`).attr('width', width + 100).attr('height', height + 100);
    var visData = new vis.DataSet();
    data.forEach(({x, y, id, energyConsumed, totalPacketReceived}) => {
      let res = 0;
      if (totalPacketReceived > thresholdDraw) res = totalPacketReceived;
      else if (totalPacketReceived > 5) res = totalPacketReceived + additive - 5;
      else res = 0;
      visData.add({x, y, id, z: res, style: res})
    });

    console.log(max);
    // specify options
    var options = {
      width: `${width + 50}px`,
      height: `${height + 50}px`,
      zMax: max + 5,
      style: 'dot-line',
      showPerspective: true,
      dotSizeRatio: 0.015,
      showGrid: true,
      showShadow: false,
      keepAspectRatio: true,
      verticalRatio: 0.5
    };

    // Instantiate our graph object.
    var container = document.getElementById(`surface${i}`);
    var graph3d = new vis.Graph3d(container, visData, options);


    let sortedLifetime = data.map(({estimateLifetime}) => estimateLifetime).sort();
    let lifeTime = sortedLifetime[0] * 24;
    let shortestPathRatioData = data.filter(({sumHopRatio, endPointCount}) => endPointCount > 0);
    let shortestPathRatio = (
      shortestPathRatioData
        .map(a => a.sumHopRatio)
        .reduce((acc, val) => acc + val, 0)
    ) / (
      shortestPathRatioData
        .map(a => a.endPointCount)
        .reduce((acc, val) => acc + val, 0)
    );


    let numPacketReceiveds = data.map(x => x.totalPacketReceived);
    let BI = (numPacketReceiveds.reduce((acc, val) => acc + val, 0) ** 2) / numPacketReceiveds.reduce((acc, val) => acc + val * val, 0) / numPacketReceiveds.length;
    let numPacketReceiveds1 = data.map(x => x.totalPacketReceived).filter(x => x > 0);
    let BI1 = (numPacketReceiveds1.reduce((acc, val) => acc + val, 0) ** 2) / numPacketReceiveds1.reduce((acc, val) => acc + val * val, 0) / numPacketReceiveds1.length;

    outterDiv.append(`<p>Estimated Life time: ${lifeTime} hours</p>`);
    outterDiv.append(`<p>Shortest path ratio: ${shortestPathRatio}</p>`);
    outterDiv.append(`<p>Number of communication sessions: ${numTraffic}</p>`);
    outterDiv.append(`<p>Blalancing Index: ${BI}</p>`);
    outterDiv.append(`<p>Blalancing Index 1: ${BI1}</p>`);

  });

};

$('#result').hide();
$('#option-panel').hide();
// $('#graph-container').hide();
// let width = 1200, height = 1200;
// handleStatistics([data, data1], width, height);


const generateNodes = ({width, height, V}) => {
  let nodes = [];
  let nextId = 0;
  let k = height / width;
  let x = Math.floor(Math.sqrt(V / k));

  let nCellWidth = x;
  let nCellHeight = Math.floor(k * x);
  let cellWidth = width / nCellWidth;
  let cellHeight = height / nCellHeight;

  console.log("ahihi");

  for (let i = 0; i < nCellWidth; i++) {
    for (let j = 0; j < nCellHeight; j++) {
      let startX = cellWidth * i;
      let startY = cellHeight * j;
      const x = Math.random() * cellWidth * 0.6 + startX + cellWidth * 0.2;
      const y = Math.random() * cellHeight * 0.6 + startY + cellHeight * 0.2;
      nodes.push({x, y, id: nextId});
      nextId++;
    }
  }

  return nodes;
};


$('#generate-btn').click(function () {
  const height = parseInt($('#height-input').val());
  const width = parseInt($('#width-input').val());
  const range = parseInt($('#range-input').val());
  const GRID_HEIGHT = parseInt($('#grid-height-input').val());
  const GRID_WIDTH = parseInt($('#grid-width-input').val());
  const V = parseInt($('#v-input').val());
  let nodes = generateNodes({width, height, GRID_WIDTH, GRID_HEIGHT, V});
  init({nodes, width, height, range});
});


let file = null;
$('#file-input').change(function () {
  if (this.files.length) {
    file = this.files[0];
    let reader = new FileReader();
    if (file) {

      reader.onload = function (event) {
        let {nodes, width, height, range} = JSON.parse(event.target.result);
        init({nodes, width, height, range})
      };

      reader.readAsText(file);
    }
  }
});

$('#generate-file-btn').click(function () {
  $('#file-input').trigger("click");
});


let network;

function init({nodes, width, height, range}) {
  console.log(nodes.length);
  if (network) {
    network.remove();
    $('#save-btn').off('click');
    $('#traffic-generate-btn').off('click');
    $('#export-btn').off('click');
    $('#submit-btn').off('click');
    $('#reset-btn').off('click');
    $('#addpair-btn').off('click');
  }

  $('#height-input').val(height.toString());
  $('#width-input').val(width.toString());
  $('#result').hide();
  $('#option-panel').hide();
  $('#graph-container').show();
  network = new Network({container: 'graph-container', nodes, width, height, range});
  let displayResult = (data) => {
    let shapes = data.split('\n').filter(_ => _ !== '').map(_ => JSON.parse(_));
    shapes.forEach(shape => {
      if (shape.type === 'circle') {
        let {centerX, centerY, radius, color} = shape;
        network.drawCircle({centerX, centerY, radius, style: {color, width: 0.5}})
      } else if (shape.type === 'point') {
        let {x, y, color} = shape;
        network.drawPoint({x, y, color});
      } else if (shape.type === 'line') {
        let {x1, y1, x2, y2, color} = shape;
        network.drawLine({x1, y1, x2, y2, style: {color, width: '0.5'}})
        // network.drawArrow(({from: {x: x1, y: y1}, to: {x: x2, y: y2}, style: {color, width: '1'}}))
      } else if (shape.type === 'arc') {
        let {fromX, fromY, toX, toY, radius, color} = shape;
        network.drawArc({from: {x: fromX, y: fromY}, to: {x: toX, y: toY}, radius, style: {width: '0.5', color}})
      }
    });
  };

  let traffic = [];

  $('#addpair-btn').click(() => {
    let pair = {
      source: null,
      destination: null,
      numPacket: 1,
      rate: 1000,
    };


    let listener = (node) => {
      let color = '#5590ff';
      network.drawHalo({
        x: node.x, y: node.y, radius: 10, color: color
      });
      if (pair.source === null) {
        pair.source = node.id;
      } else {
        pair.destination = node.id;
        traffic.push(pair);
        network.drawArrow({
          from: network.nodes[pair.source],
          to: network.nodes[pair.destination],
          style: {width: '0.5', color: color}
        })
        network.removeNodeClickListener(listener);
      }
    };

    network.addNodeClickListener(listener);
  });


  let genTraffic = (numPair) => {
    let A = network.getA();
    let B = network.getB();
    let res = [];
    while (res.length < numPair) {
      let fromId = A[Math.floor(Math.random() * A.length)].id;
      let toId = B[Math.floor(Math.random() * B.length)].id;

      if (!res.find(x => x.source === fromId || x.destination === toId)) {
        res.push({
          source: fromId,
          destination: toId,
        })
      }
    }

    return res;
  };
  let addTraffic = (base, numAdded) => {

    let res = [];
    for (let i = 0; i < base.length; i++) {
      res.push(base[i]);
    }

    let A = network.getA();
    let B = network.getB();
    while (res.length < base.length + numAdded) {
      let fromId = A[Math.floor(Math.random() * A.length)].id;
      let toId = B[Math.floor(Math.random() * B.length)].id;

      if (!res.find(x => x.source === fromId || x.destination === toId)) {
        res.push({
          source: fromId,
          destination: toId,
        })
      }
    }

    return res;
  };

  let submitReal = () => {
    // let algorithms = ['gpsr', 'rollingBall', 'shortestPath', 'stable'];
    let algorithms = ['stable', 'mlp'];
    let accData = [];
    let ts = [];
    ts.push(genTraffic(15));
    for (let i = 0; i < 0; i++) {
      ts.push(addTraffic(ts[ts.length - 1], 5));
    }

    let dateString = (new Date()).getUTCMilliseconds().toString();

    let scenes = [];
    for (let i = 0; i < algorithms.length; i++) {
      for (let j = 0; j < ts.length; j++) {
        scenes.push({
          algorithm: algorithms[i],
          traffic: ts[j],
          numTraffic: ts[j].length,
        })
      }
    }

    console.log(ts);

    let next = 0;
    let marked = {};
    let makeRequest = () => {
      console.log("makign request");
      console.log("next val", next);
      // let routingAlgorithm = algorithms[next];
      let {algorithm, traffic, numTraffic} = scenes[next];
      let data = {
        network: {
          fieldWidth: width,
          fieldHeight: height,
          nodes: network.nodes.map(({x, y}, i) => ({x, y, id: i})),
          nodeConfig: {
            routingAlgorithm: algorithm
          }
        },
        simulation: {
          traffic,
          timeLimit: 1000,
        },
        mode: 'real'
      };
      $.ajax({
        url: 'http://localhost:8080/exec',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify(data),
        success({sessionId}) {
          const interval = setInterval(() => {
            console.log("calling interval", algorithm, numTraffic);
            $.ajax({
              url: `http://localhost:8080/status/${sessionId}`,
              success({status}) {
                if (status === 'completed') {
                  clearInterval(interval);
                  $.ajax({
                    url: `http://localhost:8080/result/statistics/${sessionId}`,
                    success: (data) => {

                      if (!marked[[algorithm, numTraffic]]) {
                        marked[[algorithm, numTraffic]] = true;
                      } else {
                        return;
                      }


                      let hahaData = {
                        routingAlgorithm: algorithm,
                        numTraffic: numTraffic,
                        traffic: traffic,
                        data: JSON.parse(data),
                      };

                      download(`${dateString}_${algorithm}_${numTraffic}.json`, JSON.stringify(hahaData));


                      accData.push(hahaData);

                      if (next === scenes.length - 1) {
                        handleStatistics(accData, width, height);
                      } else {
                        next++;
                        setTimeout(() => makeRequest(), 1000);
                      }
                    },
                  });
                }
              }
            })
          }, 500);
        }
      });
    };
    makeRequest();
  };
  $('#submit-btn').click(() => {
    $('#result').hide();
    $('#option-panel').hide();
    let mode = $('#mode-input').val();
    if (mode === "debug") {
      const routingAlgorithm = $('#ra-input').val();
      let data = {
        network: {
          fieldWidth: width,
          fieldHeight: height,
          nodes: network.nodes.map(({x, y}, i) => ({x, y, id: i})),
          nodeConfig: {
            routingAlgorithm
          }
        },
        simulation: {
          traffic,
          timeLimit: 1000,
        },
        mode
      };

      $.ajax({
        url: 'http://localhost:8080/exec',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify(data),
        success({sessionId}) {
          console.log(sessionId);
          const interval = setInterval(() => {
            $.ajax({
              url: `http://localhost:8080/status/${sessionId}`,
              success({status}) {
                if (status === 'completed') {
                  clearInterval(interval);
                  $.ajax({
                    url: `http://localhost:8080/result/draw/${sessionId}`,
                    success: displayResult,
                  });
                  $.ajax({
                    url: `http://localhost:8080/result/log/${sessionId}`,
                    success: (data) => console.log(data)
                  });
                }
              }
            })
          }, 400);
        }
      });
    }
    else {
      submitReal();
    }
  });

  $('#save-btn').click(() => {
    let blob = new Blob([JSON.stringify({
      width,
      height,
      range,
      nodes: network.nodes.map(({x, y, id}) => ({x, y, id}))
    })]);
    FileSaver.saveAs(blob, "network.json");
  });

  $('#reset-btn').click(() => {
    init({nodes: network.nodes, width, height, range})
  });


  $('#export-btn').click(() => {
    let svg = network.getSVG();
    download("image.svg", svg.outerHTML);
  })
}

function download(filename, text) {
  var pom = document.createElement('a');
  pom.setAttribute('href', 'data:text/plain;charset=utf-8,' + encodeURIComponent(text));
  pom.setAttribute('download', filename);

  if (document.createEvent) {
    var event = document.createEvent('MouseEvents');
    event.initEvent('click', true, true);
    pom.dispatchEvent(event);
  }
  else {
    pom.click();
  }
}

