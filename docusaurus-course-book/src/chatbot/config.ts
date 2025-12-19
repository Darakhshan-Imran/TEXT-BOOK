/**
 * Chatbot configuration settings.
 */

// Backend API URL - Railway deployment
// For local chatbot backend testing, change to: 'http://localhost:8080'
export const CHATBOT_API_URL = 'https://text-book-production.up.railway.app';

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
