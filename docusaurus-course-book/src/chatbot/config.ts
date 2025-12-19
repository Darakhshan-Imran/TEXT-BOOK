/**
 * Chatbot configuration settings.
 */

// Backend API URL - Railway deployment
const CHATBOT_API_URL_PROD = 'https://text-book-production.up.railway.app';
const CHATBOT_API_URL_DEV = 'http://localhost:8080';

// Determine if we're in production (GitHub Pages)
const isProduction = typeof window !== 'undefined' &&
  window.location.hostname === 'darakhshan-imran.github.io';

export const CHATBOT_API_URL = isProduction ? CHATBOT_API_URL_PROD : CHATBOT_API_URL_DEV;

/**
 * Get full API URL for chatbot endpoints
 */
export const getChatbotApiUrl = (endpoint: string): string => {
  return `${CHATBOT_API_URL}${endpoint}`;
};

export const chatbotConfig = {
  apiUrl: CHATBOT_API_URL,
  endpoints: {
    query: '/chat/query',
    health: '/chat/health',
  },
  maxMessageLength: 500,
  minMessageLength: 3,
};
