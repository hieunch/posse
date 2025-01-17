import Network from "./Network";

require('svg.draggable.js');
require('svg.panzoom.js');
import fs from 'fs'
import SVG from 'svg.js';
import Mousetrap from 'mousetrap';
import inPolygon from 'point-in-polygon';
import convexHull from 'convex-hull';
import FileSaver from 'file-saver';
import SVGtoPNG from 'save-svg-as-png';
import distanceToLineSegment from 'distance-to-line-segment';
import simpleheat from 'simpleheat';
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

function downloadCanvas(canvas, filename) {

    /// create an "off-screen" anchor tag
    var lnk = document.createElement('a'),
        e;

    /// the key here is to set the download attribute of the a tag
    lnk.download = filename;

    /// convert canvas content to data-uri for link. When download
    /// attribute is set the content pointed to by link will be
    /// pushed as "download" in HTML5 capable browsers
    lnk.href = canvas.toDataURL();

    /// create a "fake" click-event to trigger the download
    if (document.createEvent) {

        e = document.createEvent("MouseEvents");
        e.initMouseEvent("click", true, true, window,
                         0, 0, 0, 0, 0, false, false, false,
                         false, 0, null);

        lnk.dispatchEvent(e);

    } else if (lnk.fireEvent) {

        lnk.fireEvent("onclick");
    }
}

let run_num = 0;
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
    $('#result').append(`<div id="result${run_num}"></div>`);
    let outterDiv = $(`#result${run_num}`);
    outterDiv.append(`<p>${routingAlgorithm}</p>`);
    outterDiv.append(`<canvas id="canvas${run_num}">${routingAlgorithm}</canvas>`);
    $(`#canvas${run_num}`).attr('width', width*1 + 100).attr('height', height*1 + 100);
    let heat = simpleheat(`canvas${run_num}`);
    run_num++;

    // function heatMapColorforValue(value){
    //   var h = (1.0 - value) * 240
    //   return "hsl(" + h + ", 100%, 50%)";
    // }

    // let b2rGradient = {};
    // for (let i = 0; i <= 1; i+=0.01) {
    //   b2rGradient[i] = heatMapColorforValue(i);
    // }
    // heat.gradient(b2rGradient);
    // heat.radius(20, 10);

    heat.gradient({
      0.6: 'blue',
      0.70: 'cyan',
      0.80: 'lime',
      0.85: 'greenyellow',
      0.90: 'yellow',
      0.95: 'orange',
      1.0: 'red'
    });
    // heat.radius(20, 0);

    let thresholdMax = 1000;
    let thresholdDraw = 80;
    let additive = 0*thresholdMax / 5;
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
    // var container = document.getElementById(`surface${i}`);
    // var graph3d = new vis.Graph3d(container, visData, options);


    let sortedLifetime = data.map(({estimateLifetime}) => estimateLifetime).sort();
    let lifeTime = sortedLifetime[0] * 24;
    let shortestPathRatioData = data.filter(({sumHopRatio, endPointCount}) => endPointCount > 0);
    let maxRatioData = data.filter(({maxRatio, endPointCount}) => endPointCount > 0);
    let shortestDistanceRatioData = data.filter(({sumDistanceRatio, endPointCount}) => endPointCount > 0);
    let shortestPathRatio = (
      shortestPathRatioData
        .map(a => a.sumHopRatio)
        .reduce((acc, val) => acc + val, 0)
    ) / (
      shortestPathRatioData
        .map(a => a.endPointCount)
        .reduce((acc, val) => acc + val, 0)
    );
    let maxRatio = (
      maxRatioData
        .map(a => a.maxRatio)
        .reduce((acc, val) => Math.max(acc, val), 0)
    );
    let shortestDistanceRatio = (
      shortestDistanceRatioData
        .map(a => a.sumDistanceRatio)
        .reduce((acc, val) => acc + val, 0)
    ) / (
      shortestDistanceRatioData
        .map(a => a.endPointCount)
        .reduce((acc, val) => acc + val, 0)
    );


    let numPacketReceiveds = data.map(x => x.totalPacketReceived);
    let total = numPacketReceiveds.reduce((acc, val) => acc + val, 0);
    let BI = (numPacketReceiveds.reduce((acc, val) => acc + val, 0) ** 2) / numPacketReceiveds.reduce((acc, val) => acc + val * val, 0) / numPacketReceiveds.length;
    let numPacketReceiveds1 = data.map(x => x.totalPacketReceived).filter(x => x > 0);
    let BI1 = (numPacketReceiveds1.reduce((acc, val) => acc + val, 0) ** 2) / numPacketReceiveds1.reduce((acc, val) => acc + val * val, 0) / numPacketReceiveds1.length;

    let endPointCounts = data.map(x => x.endPointCount);
    let totalEndPointCounts = endPointCounts.reduce((acc, val) => acc + val, 0);

    outterDiv.append(`<p>Estimated Life time: ${lifeTime} hours</p>`);
    outterDiv.append(`<p>Shortest path ratio: ${shortestPathRatio}</p>`);
    outterDiv.append(`<p>Maximum ratio: ${maxRatio}</p>`);
    outterDiv.append(`<p>Shortest distance ratio: ${shortestDistanceRatio}</p>`);
    outterDiv.append(`<p>Number of communication sessions: ${numTraffic}</p>`);
    outterDiv.append(`<p>Total packet received: ${total}</p>`);
    outterDiv.append(`<p>Total end point counts: ${totalEndPointCounts}</p>`);
    outterDiv.append(`<p>Blalancing Index: ${BI}</p>`);
    outterDiv.append(`<p>Blalancing Index 1: ${BI1}</p>`);

  });

  // let alg = ['mlp', 'stable', 'gpsr', 'shortestPath', 'vhr'];
  // for (let i=0; i<5; i++){
  //   let canvas = document.getElementById(`canvas${i}`);
  //   downloadCanvas(canvas, `${alg[i]}.png`);
  // }
  // let svg = network.getSVG();
  // download("hole.svg", svg.outerHTML);

};

$('#result').hide();
$('#option-panel').hide();
// $('#graph-container').hide();
// let width = 1200, height = 1200;
// handleStatistics([data, data1], width, height);

var cellWidth = 0;
var cellHeight = 0;
const generateNodes = ({width, height, V}) => {
  let nodes = [];
  let nextId = 0;
  let k = height / width;
  let x = Math.floor(Math.sqrt(V / k));

  let nCellWidth = x;
  let nCellHeight = Math.floor(k * x);
  cellWidth = width / nCellWidth;
  cellHeight = height / nCellHeight;

  console.log("ahihi");
  $('#nCellWidth').val(nCellWidth.toString());
  $('#nCellHeight').val(nCellHeight.toString());

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
        let {nodes, width, height, cellW, cellH, range} = JSON.parse(event.target.result);
        init({nodes, width, height, range})
        cellWidth = cellW;
        cellHeight = cellH;
      };

      reader.readAsText(file);
    }
  }
});

file = null;
$('#file-input2').change(function () {
  if (this.files.length) {
    file = this.files[0];
    let reader = new FileReader();
    if (file) {
      reader.onload = function (event) {
        let data = JSON.parse(event.target.result);
        let points = data.features[0].geometry.coordinates[0];

        let maxSize = 1200;
        let ratio = 1;
        let span = 300*maxSize/800;
        points.forEach(point => {
          let point0 = point[1];
          point[0] = point[0]/ratio;
          point[1] = point0/ratio;
        });

        let x_min = Number.MAX_VALUE;
        let y_min = Number.MAX_VALUE;
        let x_max = -Number.MAX_VALUE;
        let y_max = -Number.MAX_VALUE;
        points.forEach(point => {
          if (point[0] < x_min) x_min = point[0];
          if (point[1] < y_min) y_min = point[1];
          if (point[0] > x_max) x_max = point[0];
          if (point[1] > y_max) y_max = point[1];
        });
        console.log(x_min + " " + x_max);
        console.log(y_min + " " + y_max);

        let scale = maxSize/Math.max(x_max-x_min, y_max-y_min);
        const width = (x_max-x_min)*scale+span*3/4+span*3/4;
        const height = (y_max-y_min)*scale+span*3/4+span*3/4;
        const range = 40;
        // const V = height*width/(0.433 * range**2);
        const V = width*height/400;
        // const V = Math.pow(Math.max(height, width)/17.88, 2);
        let nodes = generateNodes({width, height, V});
        init({nodes, width, height, range});

        points.forEach(point => {
          point[0] = (point[0]-x_min)*scale+span*3/4;
          point[1] = (y_max-point[1])*scale+span*3/4;
          console.log(point[0] + " " + point[1]);
        });

        network.deleteHole(points);
      };

      reader.readAsText(file);
    }
  }
});

$('#generate-file-btn').click(function () {
  $('#file-input').trigger("click");
});

$('#load-hole-btn').click(function () {
  $('#file-input2').trigger("click");
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
    $('#addregionpair-btn').off('click');
  }

  let mode = $('#mode-input').val();
  if (mode === "debug") {
    $('#addregionpair-btn').hide();
    $('#traffic-input').hide();
    $('#addpair-btn').show();
  } else {
    $('#addregionpair-btn').show();
    $('#traffic-input').show();
    $('#addpair-btn').hide();
  }
  $('#mode-input').change(e => {
    let mode = e.target.value;
    if (mode === "debug") {
      $('#addregionpair-btn').hide();
      $('#traffic-input').hide();
      $('#addpair-btn').show();
    } else {
      $('#addregionpair-btn').show();
      $('#traffic-input').show();
      $('#addpair-btn').hide();
    }
  });


  $('#height-input').val(height.toString());
  $('#width-input').val(width.toString());
  $('#v-input').val(nodes.length.toString());
  $('#result').hide();
  $('#option-panel').hide();
  $('#graph-container').show();
  network = new Network({container: 'graph-container', nodes, width, height, range});
  let listener = (node) => {
      let color = '#FF0000';//#5590ff';
      network.drawHalo({
        x: node.x*1, y: node.y*1, radius: 12, color: color
      });
    };

  network.addNodeClickListener(listener);
  let displayResult = (data) => {
    let shapes = data.split('\n').filter(_ => _ !== '').map(_ => JSON.parse(_));
    shapes.forEach(shape => {
      if (shape.type === 'circle') {
        let {centerX, centerY, radius, color} = shape;
        centerX = centerX*1;
        centerY = centerY*1;
        radius = radius*1;
        network.drawCircle({centerX, centerY, radius, style: {color, width: 0.5, opacity:0.5}})
      } else if (shape.type === 'point') {
        let {x, y, color} = shape;
        x = x*1;
        y = y*1;
        network.drawPoint({x, y, color});
      } else if (shape.type === 'line') {
        let {x1, y1, x2, y2, color} = shape;
        x1 = x1*1;
        y1 = y1*1;
        x2 = x2*1;
        y2 = y2*1;
        network.drawLine({x1, y1, x2, y2, style: {color, width: '1'}})
        // network.drawArrow(({from: {x: x1, y: y1}, to: {x: x2, y: y2}, style: {color, width: '1'}}))
      } else if (shape.type === 'arc') {
        let {fromX, fromY, toX, toY, radius, color} = shape;
        fromX = fromX*1;
        fromY = fromY*1;
        toX = toX*1;
        toY = toY*1;
        radius = radius*1;
        network.drawArc({from: {x: fromX, y: fromY}, to: {x: toX, y: toY}, radius, style: {width: '0.5', color}})
      }
    });
  };

  $('#display-btn').click(() => {
    $.ajax({
      url: `http://localhost:8080/result/draw/00000`,
      success: displayResult,
    });
  });

  $('#statistic-btn').click(() => { 
    let scenes = ['4BFOp', '1iGB0', '485F8'];
    let algorithms = ['mlpv2', 'ich', 'stable', 'gpsr', 'rainbow'];
    let numTraffic = 15;
    let accData = [];
    console.log(scenes);

    scenes.forEach((sessionId, i) => {
      $.ajax({
        url: `http://localhost:8080/result/statistics/${sessionId}`,
        success: (data) => {

          let hahaData = {
            routingAlgorithm: 'agorithm',
            numTraffic: numTraffic,
            data: JSON.parse(data),
          };

          accData.push(hahaData);
          if (i == scenes.length-1) handleStatistics(accData, width, height);
        },
      });
    });
  });

  let traffic = [];
  $('#addpair-btn').click(() => {
    let mode = $('#mode-input').val();

    if (mode === "real") {
      alert("not applicable in real mode!")
      return;
    }

    let pair = {
      source: null,
      destination: null,
      numPacket: 1,
      rate: 1000,
    };


    let listener = (node) => {
      let color = '#FF0000';//#5590ff';
      network.drawHalo({
        x: node.x*1, y: node.y*1, radius: 12, color: color
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

  let regionTraffic = {}; // {from: {1, 2, 3}, to: {4, 5, 6}}
  let regionTrafficCount = 0;
  $('#addregionpair-btn').click(() => {

    let mode = $('#mode-input').val();
    if (mode === "debug") {
      alert("not applicable in debug mode!");
      return;
    }


    regionTrafficCount++;
    let doneA = false;
    let doneB = false;
    let isDowning = false;
    let polylineA = null;
    let polylineB = null;
    network.draw.panZoom(false);

    let mousedownListener = ({x, y}) => {
      if (polylineA === null && polylineB === null) {
        polylineA = network.selectLayer.polyline([x, y]).fill('#dfffd6').stroke({width: 0.5});
        isDowning = true;
      } else if (polylineA !== null && polylineB === null) {
        polylineB = network.selectLayer.polyline([x, y]).fill('#cfd4ff').stroke({width: 0.5});
        isDowning = true;
      }
    };

    let mousemoveListener = ({x, y}) => {
      if (isDowning) {
        if (polylineA != null && !doneA) {
          polylineA.plot(polylineA.array().value.concat([[x, y]])).fill('#dfffd6').stroke({width: 0.5});
        } else if (polylineB != null && !doneB) {
          polylineB.plot(polylineB.array().value.concat([[x, y]])).fill('#cfd4ff').stroke({width: 0.5});
        }
      }
    };

    let mouseupListener = () => {
      isDowning = false;
      if (polylineA != null && !doneA) {
        doneA = true;
      } else if (polylineB != null && !doneB) {
        doneB = true;

        let regionA = network.nodes.filter(node => {
          return inPolygon([node.x*1, node.y*1], polylineA.array().value)
        });
        let regionB = network.nodes.filter(node => {
          return inPolygon([node.x*1, node.y*1], polylineB.array().value)
        });

        let centerA = regionA.reduce((acc, val) => {
          return {
            x: acc.x + val.x,
            y: acc.y + val.y,
          }
        }, {x: 0, y: 0});
        centerA.x = centerA.x / regionA.length;
        centerA.y = centerA.y / regionA.length;
        let centerB = regionB.reduce((acc, val) => {
          return {
            x: acc.x + val.x,
            y: acc.y + val.y,
          }
        }, {x: 0, y: 0});
        centerB.x = centerB.x / regionB.length;
        centerB.y = centerB.y / regionB.length;

        network.drawText({text: regionTrafficCount.toString(), x: centerA.x, y: centerA.y});
        network.drawText({text: regionTrafficCount.toString(), x: centerB.x, y: centerB.y});
        network.drawArrow(({from: centerA, to: centerB, style: {color: 'red', width: '1'}}));


        regionTraffic[regionTrafficCount] = {
          from: regionA.map(_ => _.id),
          to: regionB.map(_ => _.id),
        };


        let trafficText = Object.keys(regionTraffic).map(_ => _ + ":").join("\n");
        $('#traffic-input').val(trafficText);

        network.removeMousedownListener(mousedownListener);
        network.removeMousemoveListener(mousemoveListener);
        network.removeMouseupListener(mouseupListener);
        network.draw.panZoom();
      }
    }

    network.addMousedownListener(mousedownListener);
    network.addMousemoveListener(mousemoveListener);
    network.addMouseupListener(mouseupListener);
  });

  let genTraffic = () => {
    // let A = network.getA();
    // let B = network.getB();
    // let res = [];
    // while (res.length < numPair) {
    //   let fromId = A[Math.floor(Math.random() * A.length)].id;
    //   let toId = B[Math.floor(Math.random() * B.length)].id;
    //
    //   if (!res.find(x => x.source === fromId || x.destination === toId)) {
    //     res.push({
    //       source: fromId,
    //       destination: toId,
    //     })
    //   }
    // }
    //
    // return res;
    let trafficGroups = $('#traffic-input').val()
      .split('\n')
      .filter(l => {
        return l && l !== "";
      })
      .map(txt => {
        let [region, numPair] = txt.split(":").map(_ => parseInt(_));
        if (isNaN(numPair)) numPair = 10;
        //numPair = N;
        return {
          region, numPair
        }
      });

    let result = [];
    trafficGroups.forEach(({region, numPair}) => {
      let A = regionTraffic[region].from;
      let B = regionTraffic[region].to;
      // if (numPair > A.length || numPair > B.length) return;
      let res = [];
      while (res.length < numPair) {
        let fromId = A[Math.floor(Math.random() * A.length)];
        let toId = B[Math.floor(Math.random() * B.length)];

        if (!res.find(x => x.source === fromId)) {;// || x.destination === toId)) {
          res.push({
            source: fromId,
            destination: toId,
          })
        }
      }

      result = [...result, ...res];
    });

    console.log(result);
    return result;
  };
  let addTraffic = (base, numAdded) => {

    let res = [];
    for (let i = 0; i < base.length; i++) {
      res.push(base[i]);
    }

    let A = regionTraffic[1].from;
    let B = regionTraffic[1].to;
    while (res.length < base.length + numAdded) {
      let fromId = A[Math.floor(Math.random() * A.length)];
      let toId = B[Math.floor(Math.random() * B.length)];

      // if (!res.find(x => x.source === fromId || x.destination === toId)) {
        res.push({
          source: fromId,
          destination: toId,
        })
      // }
    }

    return res;
  };

  let submitReal = () => {
    // for (let i = 0; i < algorithms.length; i++) {
    // let algorithms = ['gpsr', 'rollingBall', 'shortestPath', 'stable'];
    // let algorithms = ['mlpv2', 'mlp', 'stable', 'gpsr', 'shortestPath', 'VHR'];
    let algorithms = ['mlpv2'];//'mlpv2', 'stable', 'ich'];
    let accData = [];
    let ts = [];
    ts.push(genTraffic());

    // return; // TODO
    for (let i = 0; i < 1; i++) {
      ts.push(addTraffic(ts[ts.length - 1], 5));
    }

    let dateString = (new Date()).getUTCMilliseconds().toString();

    let scenes = [];
    for (let j = 0; j < ts.length; j++) {
      for (let i = 0; i < algorithms.length; i++) {
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
      console.log(cellWidth); console.log(cellHeight);
      let data = {
        network: {
          fieldWidth: width,
          fieldHeight: height,
          cellWidth: cellWidth,
          cellHeight: cellHeight,
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

                      // download(`${dateString}_${algorithm}_${numTraffic}.json`, JSON.stringify(hahaData));


                      accData.push(hahaData);

                      // handleStatistics(accData, width, height);
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
          cellWidth: cellWidth,
          cellHeight: cellHeight,
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
    let cellW = cellWidth;
    let cellH = cellHeight;
    let blob = new Blob([JSON.stringify({
      width,
      height,
      cellW,
      cellH,
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

