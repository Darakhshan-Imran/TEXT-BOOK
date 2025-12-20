// Configuration for better-auth integration
// This file defines how better-auth will connect to our backend

export const authConfig = {
  // API endpoints for authentication
  endpoints: {
    signIn: '/auth/login', // This should match your existing FastAPI endpoint
    signUp: '/auth/register', // This should match your existing FastAPI endpoint
    signOut: '/auth/logout', // This should match your existing FastAPI endpoint
    session: '/auth/profile', // This should match your existing FastAPI endpoint
  },

  // Backend API URL - always use production for now
  // For local auth backend testing, change to: 'http://127.0.0.1:8000'
  baseUrl: 'https://textbook-auth-api.onrender.com',  // Production (Render)

  // Token configuration
  token: {
    cookieName: 'auth_token',
    expiresIn: '7d', // Token expiration
  },

  // Field mappings to match your existing backend
  fieldMappings: {
    email: 'email',
    password: 'password',
    firstName: 'first_name',
    lastName: 'last_name',
  },
};

// Export a function to get the full API URL
export const getApiUrl = (endpoint: string): string => {
  return `${authConfig.baseUrl}${endpoint}`;
};