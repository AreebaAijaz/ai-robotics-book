/**
 * useAuth - Custom hook for accessing authentication context.
 */
import { useContext } from 'react';
import { AuthContext } from '../components/AuthProvider';

/**
 * Hook to access authentication state and methods.
 *
 * @returns {Object} Auth context with:
 *   - user: Current user object or null
 *   - session: Current session data or null
 *   - isLoading: Boolean indicating auth operation in progress
 *   - isAuthenticated: Boolean indicating if user is logged in
 *   - error: Error message or null
 *   - signUp: Function to register new user
 *   - signIn: Function to login user
 *   - signOut: Function to logout user
 *   - forgotPassword: Function to request password reset
 *   - resetPassword: Function to reset password with token
 *   - clearError: Function to clear error state
 *   - getToken: Function to get current auth token
 *
 * @example
 * const { user, isAuthenticated, signIn, signOut } = useAuth();
 *
 * if (isAuthenticated) {
 *   return <div>Welcome, {user.email}!</div>;
 * }
 */
export default function useAuth() {
  const context = useContext(AuthContext);

  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }

  return context;
}
