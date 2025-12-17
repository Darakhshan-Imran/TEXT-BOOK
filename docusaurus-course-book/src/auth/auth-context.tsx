import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import { authClient } from './client';

interface AuthContextType {
  user: any;
  signIn: (email: string, password: string) => Promise<any>;
  signUp: (email: string, password: string, firstName?: string, lastName?: string) => Promise<any>;
  signOut: () => Promise<void>;
  isLoading: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<any>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Check if user is authenticated on mount
    const checkAuthStatus = async () => {
      try {
        const session = await authClient.getSession();
        if (session) {
          setUser(session.user);
        }
      } catch (error) {
        console.error('Error checking auth status:', error);
      } finally {
        setIsLoading(false);
      }
    };

    checkAuthStatus();
  }, []);

  const signIn = async (email: string, password: string) => {
    try {
      const response = await authClient.signIn.email({
        email,
        password,
        callbackURL: "/TEXT-BOOK/docs", // Redirect after successful login
      });

      if (response?.success && response?.user) {
        setUser(response.user);
        return { success: true, user: response.user };
      }

      return { success: false, error: response?.error?.message || "Sign in failed" };
    } catch (error) {
      console.error('Sign in error:', error);
      return { success: false, error: (error as Error).message || "Sign in failed" };
    }
  };

  const signUp = async (email: string, password: string, firstName?: string, lastName?: string) => {
    try {
      const response = await authClient.signUp.email({
        email,
        password,
        firstName,
        lastName,
        callbackURL: "/TEXT-BOOK/docs", // Redirect after successful signup
      });

      if (response?.success && response?.user) {
        setUser(response.user);
        return { success: true, user: response.user };
      }

      return { success: false, error: response?.error?.message || "Sign up failed" };
    } catch (error) {
      console.error('Sign up error:', error);
      return { success: false, error: (error as Error).message || "Sign up failed" };
    }
  };

  const signOut = async () => {
    try {
      await authClient.signOut();
      setUser(null);
    } catch (error) {
      console.error('Sign out error:', error);
    }
  };

  return (
    <AuthContext.Provider value={{ user, signIn, signUp, signOut, isLoading }}>
      {children}
    </AuthContext.Provider>
  );
};