// Expose environment variables to client-side
if (typeof window !== 'undefined') {
  window.REACT_APP_CHAT_API_URL = process.env.REACT_APP_CHAT_API_URL || 'http://localhost:3001/api/chat';
}