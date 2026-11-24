/**
 * NavbarLoginButton - Login/Signup button for the navbar.
 * Uses React portal to render into the navbar HTML placeholder.
 */
import React, { useState, useEffect } from 'react';
import { createPortal } from 'react-dom';
import useAuth from '../../hooks/useAuth';
import styles from './styles.module.css';

/**
 * The actual button content.
 */
function LoginButtonContent() {
  const { isAuthenticated, user, signOut, isLoading } = useAuth();

  const handleLoginClick = () => {
    // Dispatch custom event to open auth modal
    window.dispatchEvent(new CustomEvent('openAuthModal'));
  };

  const handleSignOut = () => {
    signOut();
  };

  // Loading state
  if (isLoading) {
    return null;
  }

  // Authenticated state
  if (isAuthenticated && user) {
    return (
      <div className={styles.userMenu}>
        <span className={styles.userEmail}>
          {user.email.split('@')[0]}
        </span>
        <button
          className={styles.signOutButton}
          onClick={handleSignOut}
          title="Sign out"
        >
          Sign Out
        </button>
      </div>
    );
  }

  // Not authenticated
  return (
    <button
      className={styles.loginButton}
      onClick={handleLoginClick}
    >
      <svg className={styles.loginIcon} viewBox="0 0 24 24" width="18" height="18">
        <path d="M12 12c2.21 0 4-1.79 4-4s-1.79-4-4-4-4 1.79-4 4 1.79 4 4 4zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
      </svg>
      <span>Login</span>
    </button>
  );
}

/**
 * Navbar login button component.
 * Uses portal to render into #navbar-login-button div.
 */
export default function NavbarLoginButton() {
  const [container, setContainer] = useState(null);

  useEffect(() => {
    // Docusaurus re-renders the navbar on client-side route changes, replacing
    // the placeholder div. Re-locate it whenever the DOM changes so the portal
    // never stays attached to a detached node (which made the button vanish).
    const findContainer = () => {
      const el = document.getElementById('navbar-login-button');
      setContainer((prev) => (prev === el ? prev : el));
    };

    findContainer();
    const observer = new MutationObserver(findContainer);
    observer.observe(document.body, { childList: true, subtree: true });
    return () => observer.disconnect();
  }, []);

  // Don't render during SSR or if container not found
  if (typeof window === 'undefined' || !container) {
    return null;
  }

  return createPortal(<LoginButtonContent />, container);
}
