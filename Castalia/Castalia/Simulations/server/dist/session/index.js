"use strict";

Object.defineProperty(exports, "__esModule", {
  value: true
});
const activeSession = {};

exports.default = {
  markStatus(sessionId, status, message) {
    activeSession[sessionId] = { id: sessionId, status, message };
  },

  getSession(sessionId) {
    return activeSession[sessionId];
  }
};
//# sourceMappingURL=index.js.map