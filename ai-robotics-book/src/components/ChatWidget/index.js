import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import ChatPanel from './ChatPanel';
import AuthModal from '../AuthModal';
import ProfileSettings from '../ProfileSettings';
import useChat from '../../hooks/useChat';
import useAuth from '../../hooks/useAuth';
import styles from './styles.module.css';

/**
 * Main ChatWidget component with floating button and panel.
 * Requires authentication to access the chatbot.
 */
export default function ChatWidget({ pendingSelection, onSelectionHandled }) {
  const [isOpen, setIsOpen] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [showSettings, setShowSettings] = useState(false);
  const location = useLocation();
  const { isAuthenticated, isLoading: authLoading, user, signOut } = useAuth();
  const {
    messages,
    isLoading,
    error,
    sendMessage,
    sendSelectionQuery,
    retryLastMessage,
    clearChat
  } = useChat();

  // Detect if we're on the homepage (navy background) or docs pages (white background)
  const isHomePage = location.pathname === '/' || location.pathname === '/ai-robotics-book/' || location.pathname === '/ai-robotics-book';
  const themeClass = isHomePage ? styles.lightTheme : styles.darkTheme;

  // Handle pending selection from text selection
  useEffect(() => {
    if (pendingSelection && !isLoading && isAuthenticated) {
      // Open the chat panel
      setIsOpen(true);
      // Send the selection query
      sendSelectionQuery(pendingSelection);
      // Clear the pending selection
      if (onSelectionHandled) {
        onSelectionHandled();
      }
    } else if (pendingSelection && !isAuthenticated) {
      // Show auth modal if not authenticated
      setShowAuthModal(true);
      if (onSelectionHandled) {
        onSelectionHandled();
      }
    }
  }, [pendingSelection, isLoading, isAuthenticated, sendSelectionQuery, onSelectionHandled]);

  const handleToggle = () => {
    if (!isAuthenticated && !isOpen) {
      // Show auth modal if not authenticated
      setShowAuthModal(true);
    } else {
      setIsOpen(!isOpen);
    }
  };

  const handleClose = () => {
    setIsOpen(false);
  };

  const handleAuthSuccess = () => {
    setShowAuthModal(false);
    setIsOpen(true);
  };

  const handleCloseAuthModal = () => {
    setShowAuthModal(false);
  };

  const handleOpenSettings = () => {
    setShowSettings(true);
  };

  const handleCloseSettings = () => {
    setShowSettings(false);
  };

  // Handle backdrop click - only close if clicking directly on backdrop
  const handleBackdropClick = (e) => {
    // Only close if the click target is the backdrop itself
    if (e.target === e.currentTarget) {
      handleClose();
    }
  };

  return (
    <div className={`${styles.chatWidgetContainer} ${themeClass}`}>
      {/* Auth Modal for unauthenticated users */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={handleCloseAuthModal}
        onSuccess={handleAuthSuccess}
      />

      {/* Backdrop overlay when chat is open - inside container for proper stacking */}
      {isOpen && (
        <div
          className={styles.backdrop}
          onClick={handleBackdropClick}
          aria-hidden="true"
        />
      )}

      {isOpen && isAuthenticated && (
        <div className={styles.chatPanelWrapper}>
          <ChatPanel
            messages={messages}
            isLoading={isLoading}
            error={error}
            onSend={sendMessage}
            onClose={handleClose}
            onRetry={error ? retryLastMessage : null}
            onClear={clearChat}
            themeClass={themeClass}
            user={user}
            onSignOut={signOut}
            onOpenSettings={handleOpenSettings}
          />
        </div>
      )}

      {/* Profile Settings Modal */}
      {showSettings && (
        <div className={styles.settingsModal} role="dialog" aria-modal="true">
          <div
            className={styles.settingsBackdrop}
            onClick={handleCloseSettings}
            aria-hidden="true"
          />
          <div className={styles.settingsContent} onClick={(e) => e.stopPropagation()}>
            <ProfileSettings onClose={handleCloseSettings} />
          </div>
        </div>
      )}

      <button
        className={styles.chatButton}
        onClick={handleToggle}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
      >
        {isOpen ? (
          <svg className={styles.chatButtonIcon} viewBox="0 0 24 24">
            <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
          </svg>
        ) : (
          <svg className={styles.chatButtonIcon} viewBox="0 0 24 24">
            <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
          </svg>
        )}
      </button>
    </div>
  );
}
