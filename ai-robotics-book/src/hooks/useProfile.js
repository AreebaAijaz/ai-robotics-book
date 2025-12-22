/**
 * useProfile - Custom hook for fetching and updating user profile.
 */
import { useState, useCallback, useEffect } from 'react';
import useAuth from './useAuth';

// API base URL
const API_BASE_URL = typeof window !== 'undefined'
  ? (window.__API_BASE_URL__ || 'https://ai-robotics-book.onrender.com')
  : 'https://ai-robotics-book.onrender.com';

/**
 * Hook to manage user profile data.
 *
 * @returns {Object} Profile context with:
 *   - profile: Current profile data or null
 *   - isLoading: Boolean indicating operation in progress
 *   - error: Error message or null
 *   - fetchProfile: Function to fetch profile
 *   - updateProfile: Function to update profile
 *   - refetch: Function to refetch profile
 */
export default function useProfile() {
  const { isAuthenticated, getToken } = useAuth();
  const [profile, setProfile] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);

  /**
   * Fetch the user's profile from the API.
   */
  const fetchProfile = useCallback(async () => {
    const token = getToken();
    if (!token) {
      setProfile(null);
      return null;
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/user/profile`, {
        method: 'GET',
        headers: {
          'Authorization': `Bearer ${token}`,
        },
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail?.message || data.message || 'Failed to fetch profile');
      }

      const data = await response.json();
      setProfile(data);
      return data;
    } catch (err) {
      setError(err.message);
      return null;
    } finally {
      setIsLoading(false);
    }
  }, [getToken]);

  /**
   * Update the user's profile.
   */
  const updateProfile = useCallback(async (updates) => {
    const token = getToken();
    if (!token) {
      setError('Not authenticated');
      return { success: false, error: 'Not authenticated' };
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_BASE_URL}/api/user/profile`, {
        method: 'PUT',
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(updates),
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail?.message || data.message || 'Failed to update profile');
      }

      const data = await response.json();
      setProfile(data);
      return { success: true, profile: data };
    } catch (err) {
      setError(err.message);
      return { success: false, error: err.message };
    } finally {
      setIsLoading(false);
    }
  }, [getToken]);

  // Fetch profile when authenticated
  useEffect(() => {
    if (isAuthenticated) {
      fetchProfile();
    } else {
      setProfile(null);
    }
  }, [isAuthenticated, fetchProfile]);

  return {
    profile,
    isLoading,
    error,
    fetchProfile,
    updateProfile,
    refetch: fetchProfile,
  };
}
