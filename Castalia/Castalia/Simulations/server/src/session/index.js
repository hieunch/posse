const activeSession = {};

export default {
  markStatus(sessionId, status, message) {
  	console.log(activeSession);
    activeSession[sessionId] = {id: sessionId, status, message}
  },

  getSession(sessionId) {
  	console.log(activeSession);
    return activeSession[sessionId]
  }
}
