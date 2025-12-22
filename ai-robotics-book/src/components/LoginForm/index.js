/**
 * LoginForm - User login form with email and password.
 * Includes forgot password link and switch to signup.
 */
import React, { useState } from 'react';
import useAuth from '../../hooks/useAuth';
import styles from './styles.module.css';

/**
 * Login form component.
 */
export default function LoginForm({ onSuccess, onSwitchToSignup, onForgotPassword }) {
  const { signIn, isLoading, error, clearError } = useAuth();

  // Form state
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [validationErrors, setValidationErrors] = useState({});

  // Validate form
  const validate = () => {
    const errors = {};

    if (!email) {
      errors.email = 'Email is required';
    }
    if (!password) {
      errors.password = 'Password is required';
    }

    setValidationErrors(errors);
    return Object.keys(errors).length === 0;
  };

  // Handle form submit
  const handleSubmit = async (e) => {
    e.preventDefault();
    clearError();

    if (!validate()) {
      return;
    }

    const result = await signIn(email, password);
    if (result.success && onSuccess) {
      onSuccess(result.user);
    }
  };

  return (
    <div className={styles.loginForm}>
      <form onSubmit={handleSubmit} className={styles.form}>
        <h2 className={styles.title}>Welcome Back</h2>
        <p className={styles.subtitle}>
          Log in to access your personalized chatbot
        </p>

        {error && <div className={styles.errorBanner}>{error}</div>}

        <div className={styles.field}>
          <label htmlFor="login-email" className={styles.label}>Email</label>
          <input
            type="email"
            id="login-email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            className={`${styles.input} ${validationErrors.email ? styles.inputError : ''}`}
            placeholder="you@example.com"
            autoComplete="email"
            autoFocus
          />
          {validationErrors.email && (
            <span className={styles.errorText}>{validationErrors.email}</span>
          )}
        </div>

        <div className={styles.field}>
          <label htmlFor="login-password" className={styles.label}>Password</label>
          <input
            type="password"
            id="login-password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            className={`${styles.input} ${validationErrors.password ? styles.inputError : ''}`}
            placeholder="Enter your password"
            autoComplete="current-password"
          />
          {validationErrors.password && (
            <span className={styles.errorText}>{validationErrors.password}</span>
          )}
        </div>

        {onForgotPassword && (
          <div className={styles.forgotPasswordContainer}>
            <button
              type="button"
              onClick={onForgotPassword}
              className={styles.forgotPasswordButton}
            >
              Forgot password?
            </button>
          </div>
        )}

        <button type="submit" className={styles.submitButton} disabled={isLoading}>
          {isLoading ? 'Logging in...' : 'Log In'}
        </button>

        <p className={styles.switchText}>
          Don't have an account?{' '}
          <button type="button" onClick={onSwitchToSignup} className={styles.linkButton}>
            Sign up
          </button>
        </p>
      </form>
    </div>
  );
}
