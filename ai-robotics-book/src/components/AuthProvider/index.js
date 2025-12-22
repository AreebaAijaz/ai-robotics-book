/**
 * AuthProvider - Authentication context for the application.
 * Uses Better Auth for email/password authentication with session management.
 */
import React, { createContext, useState, useEffect, useCallback } from 'react';

// API base URL - use environment variable or default to production
const API_BASE_URL = typeof window !== 'undefined'
  ? (window.__API_BASE_URL__ || 'https://ai-robotics-chatbot.onrender.com')
  : 'https://ai-robotics-chatbot.onrender.com';

// Create the auth context
export const AuthContext = createContext(null);

/**
 * AuthProvider component that wraps the app and provides authentication state.
 */
export default function AuthProvider({ children }) {
  const [user, setUser] = useState(null);
  const [session, setSession] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);

  // Check for existing session on mount
  useEffect(() => {
    const checkSession = async () => {
      try {
        const storedSession = localStorage.getItem('auth_session');
        if (storedSession) {
          const sessionData = JSON.parse(storedSession);
          // Verify session is still valid
          if (sessionData.expiresAt && new Date(sessionData.expiresAt) > new Date()) {
            setSession(sessionData);
            setUser(sessionData.user);
          } else {
            // Session expired, clear storage
            localStorage.removeItem('auth_session');
          }
        }
      } catch (err) {
        console.error('Error checking session:', err);
        localStorage.removeItem('auth_session');
      } finally {
        setIsLoading(false);
      }
    };

    checkSession();
  }, []);

  /**
   * Sign up a new user with email, password, and profile data.
   */
  const signUp = useCallback(async (email, password, profile) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
          profile,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail?.message || data.message || 'Signup failed');
      }

      // Store session
      const sessionData = {
        token: data.token,
        user: data.user,
        expiresAt: data.expires_at,
      };
      localStorage.setItem('auth_session', JSON.stringify(sessionData));
      setSession(sessionData);
      setUser(data.user);

      return { success: true, user: data.user };
    } catch (err) {
      setError(err.message);
      return { success: false, error: err.message };
    } finally {
      setIsLoading(false);
    }
  }, []);

  /**
   * Sign in an existing user with email and password.
   */
  const signIn = useCallback(async (email, password) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail?.message || data.message || 'Login failed');
      }

      // Store session
      const sessionData = {
        token: data.token,
        user: data.user,
        expiresAt: data.expires_at,
      };
      localStorage.setItem('auth_session', JSON.stringify(sessionData));
      setSession(sessionData);
      setUser(data.user);

      return { success: true, user: data.user };
    } catch (err) {
      setError(err.message);
      return { success: false, error: err.message };
    } finally {
      setIsLoading(false);
    }
  }, []);

  /**
   * Sign out the current user.
   */
  const signOut = useCallback(async () => {
    setIsLoading(true);

    try {
      if (session?.token) {
        await fetch(`${API_BASE_URL}/api/auth/logout`, {
          method: 'POST',
          headers: {
            'Authorization': `Bearer ${session.token}`,
          },
        });
      }
    } catch (err) {
      console.error('Logout error:', err);
    } finally {
      localStorage.removeItem('auth_session');
      setSession(null);
      setUser(null);
      setError(null);
      setIsLoading(false);
    }
  }, [session]);

  /**
   * Request password reset email.
   */
  const forgotPassword = useCallback(async (email) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/forgot-password`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail?.message || data.message || 'Request failed');
      }

      return { success: true };
    } catch (err) {
      setError(err.message);
      return { success: false, error: err.message };
    } finally {
      setIsLoading(false);
    }
  }, []);

  /**
   * Reset password with token.
   */
  const resetPassword = useCallback(async (token, newPassword) => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/auth/reset-password`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          token,
          new_password: newPassword,
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail?.message || data.message || 'Reset failed');
      }

      return { success: true };
    } catch (err) {
      setError(err.message);
      return { success: false, error: err.message };
    } finally {
      setIsLoading(false);
    }
  }, []);

  /**
   * Clear any auth errors.
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  /**
   * Get the current auth token for API requests.
   */
  const getToken = useCallback(() => {
    return session?.token || null;
  }, [session]);

  const value = {
    user,
    session,
    isLoading,
    isAuthenticated: !!user,
    error,
    signUp,
    signIn,
    signOut,
    forgotPassword,
    resetPassword,
    clearError,
    getToken,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
}
