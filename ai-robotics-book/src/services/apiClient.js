/**
 * API client for RAG Chatbot backend.
 */

// Production API URL (Render deployment)
const PRODUCTION_API_URL = 'https://ai-robotics-book.onrender.com/api';
const DEVELOPMENT_API_URL = 'http://localhost:8000/api';

/**
 * Get the API base URL based on environment.
 * Uses localhost for development, Render URL for production.
 */
function getApiBaseUrl() {
  // Check if running in browser
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    // Use localhost API for local development
    if (hostname === 'localhost' || hostname === '127.0.0.1') {
      return DEVELOPMENT_API_URL;
    }
  }
  // Use production API for GitHub Pages or any other deployment
  return PRODUCTION_API_URL;
}

const API_BASE_URL = getApiBaseUrl();

/**
 * Get the auth token from localStorage.
 * @returns {string|null} The auth token or null.
 */
function getAuthToken() {
  try {
    const session = localStorage.getItem('auth_session');
    if (session) {
      const data = JSON.parse(session);
      return data.token || null;
    }
  } catch (e) {
    console.error('Error getting auth token:', e);
  }
  return null;
}

/**
 * Send a chat message to the backend.
 * @param {string} sessionId - The session ID.
 * @param {string} message - The user's message.
 * @returns {Promise<Object>} The chat response.
 */
export async function sendMessage(sessionId, message) {
  const token = getAuthToken();
  const headers = {
    'Content-Type': 'application/json',
  };

  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  const response = await fetch(`${API_BASE_URL}/chat`, {
    method: 'POST',
    headers,
    body: JSON.stringify({
      session_id: sessionId,
      message: message,
    }),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({}));
    throw new Error(error.message || `HTTP error: ${response.status}`);
  }

  return response.json();
}

/**
 * Send a selection-based chat message to the backend.
 * @param {string} sessionId - The session ID.
 * @param {string} selectedText - The text selected by the user.
 * @param {string} [question] - Optional question about the selection.
 * @returns {Promise<Object>} The chat response.
 */
export async function sendSelectionMessage(sessionId, selectedText, question = null) {
  const token = getAuthToken();
  const headers = {
    'Content-Type': 'application/json',
  };

  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  const body = {
    session_id: sessionId,
    selected_text: selectedText,
  };

  if (question) {
    body.question = question;
  }

  const response = await fetch(`${API_BASE_URL}/chat/selection`, {
    method: 'POST',
    headers,
    body: JSON.stringify(body),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({}));
    throw new Error(error.message || `HTTP error: ${response.status}`);
  }

  return response.json();
}

/**
 * Check the health of the backend API.
 * @returns {Promise<Object>} The health status.
 */
export async function healthCheck() {
  const response = await fetch(`${API_BASE_URL}/health`);

  if (!response.ok) {
    throw new Error(`Health check failed: ${response.status}`);
  }

  return response.json();
}
