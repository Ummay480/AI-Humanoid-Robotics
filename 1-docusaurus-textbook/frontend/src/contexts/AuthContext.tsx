import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { authClient } from '../lib/auth';

interface User {
  id: string;
  email: string;
  name?: string;
  role?: string;
}

interface AuthContextType {
  user: User | null;
  token: string | null;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<void>;
  register: (email: string, password: string, name?: string) => Promise<void>;
  logout: () => Promise<void>;
  refreshToken: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider = ({ children }: { children: ReactNode }) => {
  const [user, setUser] = useState<User | null>(null);
  const [token, setToken] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);

  useEffect(() => {
    // Check for existing session on component mount
    const checkSession = async () => {
      try {
        const session = await authClient.getSession();
        if (session?.user) {
          setUser({
            id: session.user.id,
            email: session.user.email,
            name: session.user.name,
          });
          // In a real implementation, you'd also get the token
          setToken(session.session?.token || null);
        }
      } catch (error) {
        console.error('Error checking session:', error);
      } finally {
        setIsLoading(false);
      }
    };

    checkSession();
  }, []);

  const login = async (email: string, password: string) => {
    try {
      const response = await authClient.signIn.email({
        email,
        password,
        callbackURL: '/', // Redirect after login
      });

      if (response?.user) {
        setUser({
          id: response.user.id,
          email: response.user.email,
          name: response.user.name,
        });
        setToken(response.session?.token || null);
      }
    } catch (error) {
      console.error('Login error:', error);
      throw error;
    }
  };

  const register = async (email: string, password: string, name?: string) => {
    try {
      const response = await authClient.signUp.email({
        email,
        password,
        name,
      });

      if (response?.user) {
        setUser({
          id: response.user.id,
          email: response.user.email,
          name: response.user.name,
        });
        setToken(response.session?.token || null);
      }
    } catch (error) {
      console.error('Registration error:', error);
      throw error;
    }
  };

  const logout = async () => {
    try {
      await authClient.signOut();
      setUser(null);
      setToken(null);
    } catch (error) {
      console.error('Logout error:', error);
    }
  };

  const refreshToken = async () => {
    // In a real implementation, you'd refresh the JWT token
    // This is a placeholder for the refresh logic
    try {
      const session = await authClient.getSession();
      if (session?.user) {
        setToken(session.session?.token || null);
      }
    } catch (error) {
      console.error('Token refresh error:', error);
    }
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        token,
        isLoading,
        login,
        register,
        logout,
        refreshToken,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};