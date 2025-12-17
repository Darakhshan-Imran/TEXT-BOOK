import { authConfig, getApiUrl } from './config';

// Define a custom auth client that works with our existing FastAPI backend
export const authClient = {
  // Sign in function
  signIn: {
    email: async (params: { email: string; password: string; callbackURL?: string }) => {
      try {
        const response = await fetch(getApiUrl('/auth/login'), {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            email: params.email,
            password: params.password,
          }),
        });

        const data = await response.json();

        if (response.ok) {
          // Store the token in localStorage
          if (data.access_token) {
            localStorage.setItem('access_token', data.access_token);

            // Fetch user profile with the new token
            const profileResponse = await fetch(getApiUrl('/auth/profile'), {
              headers: {
                'Authorization': `Bearer ${data.access_token}`,
              },
            });

            if (profileResponse.ok) {
              const user = await profileResponse.json();
              return { user, success: true };
            }
          }
          return { user: { email: params.email }, success: true };
        } else {
          return { error: { message: data.detail || 'Sign in failed' }, success: false };
        }
      } catch (error) {
        console.error('Sign in error:', error);
        return { error: { message: 'Network error' }, success: false };
      }
    },
  },

  // Sign up function
  signUp: {
    email: async (params: { email: string; password: string; firstName?: string; lastName?: string; callbackURL?: string }) => {
      try {
        const response = await fetch(getApiUrl('/auth/register'), {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            email: params.email,
            password: params.password,
            first_name: params.firstName,
            last_name: params.lastName,
          }),
        });

        const data = await response.json();

        if (response.ok) {
          // Store the token in localStorage
          if (data.access_token) {
            localStorage.setItem('access_token', data.access_token);

            // Fetch user profile with the new token
            const profileResponse = await fetch(getApiUrl('/auth/profile'), {
              headers: {
                'Authorization': `Bearer ${data.access_token}`,
              },
            });

            if (profileResponse.ok) {
              const user = await profileResponse.json();
              return { user, success: true };
            }
          }
          return { user: { email: params.email }, success: true };
        } else {
          return { error: { message: data.detail || 'Sign up failed' }, success: false };
        }
      } catch (error) {
        console.error('Sign up error:', error);
        return { error: { message: 'Network error' }, success: false };
      }
    },
  },

  // Sign out function
  signOut: async () => {
    try {
      // Remove the token from localStorage
      localStorage.removeItem('access_token');

      // Optionally call the backend logout endpoint
      await fetch(getApiUrl('/auth/logout'), {
        method: 'POST',
        headers: {
          'Authorization': `Bearer ${localStorage.getItem('access_token')}`,
        },
      });
    } catch (error) {
      console.error('Sign out error:', error);
    }
  },

  // Get session function
  getSession: async () => {
    try {
      const token = localStorage.getItem('access_token');
      if (!token) {
        return null;
      }

      const response = await fetch(getApiUrl('/auth/profile'), {
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      if (response.ok) {
        const user = await response.json();
        return { session: { token }, user };
      } else {
        // If the token is invalid, remove it
        localStorage.removeItem('access_token');
        return null;
      }
    } catch (error) {
      console.error('Get session error:', error);
      return null;
    }
  },
};

// Export SSR helper (simplified for client-side only)
export const { getSSRUser, withAuth } = {
  getSSRUser: async () => null, // Docusaurus is client-side rendered
  withAuth: (component: any) => component,
};