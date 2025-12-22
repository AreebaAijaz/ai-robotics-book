/**
 * AuthModal - Modal overlay for login/signup forms.
 * Used when unauthenticated users try to access the chatbot.
 */
import React, { useState } from 'react';
import LoginForm from '../LoginForm';
import SignupForm from '../SignupForm';
import styles from './styles.module.css';

/**
 * Auth modal that switches between login and signup.
 */
export default function AuthModal({ isOpen, onClose, onSuccess }) {
  const [mode, setMode] = useState('login'); // 'login' or 'signup'

  if (!isOpen) return null;

  const handleSuccess = (user) => {
    if (onSuccess) {
      onSuccess(user);
    }
    onClose();
  };

  const handleBackdropClick = (e) => {
    if (e.target === e.currentTarget) {
      onClose();
    }
  };

  return (
    <div className={styles.overlay} onClick={handleBackdropClick} role="dialog" aria-modal="true">
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={onClose} aria-label="Close">
          <svg viewBox="0 0 24 24" width="24" height="24">
            <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
          </svg>
        </button>

        <div className={styles.content}>
          {mode === 'login' ? (
            <LoginForm
              onSuccess={handleSuccess}
              onSwitchToSignup={() => setMode('signup')}
              onForgotPassword={() => {/* TODO: Implement forgot password modal */}}
            />
          ) : (
            <SignupForm
              onSuccess={handleSuccess}
              onSwitchToLogin={() => setMode('login')}
            />
          )}
        </div>
      </div>
    </div>
  );
}
