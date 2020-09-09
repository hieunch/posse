'use strict';

Object.defineProperty(exports, "__esModule", {
  value: true
});

var _express = require('express');

var _exec = require('./executor/exec');

var _fs = require('fs');

var _fs2 = _interopRequireDefault(_fs);

var _path = require('path');

var _path2 = _interopRequireDefault(_path);

var _session = require('./session');

var _session2 = _interopRequireDefault(_session);

function _interopRequireDefault(obj) { return obj && obj.__esModule ? obj : { default: obj }; }

const routes = (0, _express.Router)();

/**
 *
 * GET home page
 */
/**
 * GET /list
 *
 * This is a sample route demonstrating
 * a simple approach to error handling and testing
 * the global error handler. You most certainly want to
 * create different/better error handlers depending on
 * your use case.
 */
routes.post('/exec', (req, res) => {
  (0, _exec.register)(req.body, (error, simulationResult) => {
    if (error) {
      res.json({
        status: "error",
        error: error.toString()
      });
    } else {
      res.json(simulationResult);
    }
  });
});

routes.get('/status/:sessionId', (req, res) => {
  const sessionId = req.params.sessionId;
  const ss = _session2.default.getSession(sessionId) || { status: "error", error: "invalid session id" };
  if (!ss) {
    res.status(404).json(ss);
    return;
  }

  res.json(ss);
});

routes.get('/result/energy/:sessionId', (req, res, next) => {
  const sessionId = req.params.sessionId;
  const ss = _session2.default.getSession(sessionId) || { status: "error", error: "invalid session id" };
  if (ss.status !== 'completed') {
    res.status(404).json(ss);
    return;
  }

  const filePath = _path2.default.resolve(`logs/${sessionId}_energyTrace.txt`);

  res.sendFile(filePath, error => {
    if (error) {
      res.status(404).json({
        status: "error",
        error: "Invalid session id or simulation not completed"
      });
    }
  });
});

routes.get('/result/statistics/:sessionId', (req, res, next) => {
  const sessionId = req.params.sessionId;
  const ss = _session2.default.getSession(sessionId) || { status: "error", error: "invalid session id" };
  if (ss.status !== 'completed') {
    res.status(404).json(ss);
    return;
  }

  const filePath = _path2.default.resolve(`logs/${sessionId}_statistics.txt`);

  res.sendFile(filePath, error => {
    if (error) {
      res.status(404).json({
        status: "error",
        error: "Invalid session id or simulation not completed"
      });
    }
  });
});

routes.get('/result/log/:sessionId', (req, res, next) => {
  const sessionId = req.params.sessionId;
  const ss = _session2.default.getSession(sessionId) || { status: "error", error: "invalid session id" };
  if (ss.status !== 'completed') {
    res.status(404).json(ss);
    return;
  }

  const filePath = _path2.default.resolve(`logs/${sessionId}_log.txt`);

  res.sendFile(filePath, error => {
    if (error) {
      res.status(404).json({
        status: "error",
        error: "Invalid session id or simulation not completed"
      });
    }
  });
});

routes.get('/result/draw/:sessionId', (req, res, next) => {
  const sessionId = req.params.sessionId;
  const ss = _session2.default.getSession(sessionId) || { status: "error", error: "invalid session id" };
  if (ss.status !== 'completed') {
    res.status(404).json(ss);
    return;
  }

  const filePath = _path2.default.resolve(`logs/${sessionId}_draw.txt`);

  res.sendFile(filePath, error => {
    if (error) {
      res.status(404).json({
        status: "error",
        error: "Invalid session id or simulation not completed"
      });
    }
  });
});

routes.get('/result/event/:sessionId', (req, res, next) => {
  const sessionId = req.params.sessionId;
  const ss = _session2.default.getSession(sessionId) || { status: "error", error: "invalid session id" };
  if (ss.status !== 'completed') {
    res.status(404).json(ss);
    return;
  }

  const filePath = _path2.default.resolve(`logs/${sessionId}_eventTrace.txt`);

  res.sendFile(filePath, error => {
    if (error) {
      res.status(404).json({
        status: "error",
        error: "Invalid session id or simulation not completed"
      });
    }
  });
});

routes.get('/result/node/:sessionId', (req, res, next) => {
  const sessionId = req.params.sessionId;
  const filePath = _path2.default.resolve(`logs/${sessionId}_nodeTrace.txt`);
  const ss = _session2.default.getSession(sessionId) || { status: "error", error: "invalid session id" };
  if (ss.status !== 'completed') {
    res.status(404).json(ss);
    return;
  }

  _fs2.default.readFile(filePath, (err, data) => {
    if (err) {
      res.status(404).json({
        status: "error",
        error: "Invalid session id or simulation not completed"
      });
    } else {
      const nodes = data.toString().split('\n').filter(_ => _ !== '').map(_ => JSON.parse(_));

      res.json(nodes);
    }
  });
});

exports.default = routes;
//# sourceMappingURL=routes.js.map