import React, { useRef, useEffect } from 'react';
import Message from './Message';
import ChatInput from './ChatInput';
import styles from './styles.module.css';

/**
 * Chat panel with message list and input.
 */
export default function ChatPanel({
  messages,
  isLoading,
  error,
  onSend,
  onClose,
  onRetry,
  onClear,
  themeClass,
  user,
  onSignOut,
  onOpenSettings
}) {
  const messagesEndRef = useRef(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, isLoading]);

  // Handle sign out with confirmation
  const handleSignOut = () => {
    if (onSignOut) {
      onSignOut();
      onClose();
    }
  };

  return (
    <div className={`${styles.chatPanel} ${themeClass || ''}`}>
      {/* Header */}
      <div className={styles.panelHeader}>
        <div className={styles.headerLeft}>
          <h3 className={styles.panelTitle}>Book Assistant</h3>
          {user && (
            <span className={styles.userBadge} title={user.email}>
              {user.email.split('@')[0]}
            </span>
          )}
        </div>
        <div className={styles.headerButtons}>
          {onOpenSettings && (
            <button
              className={styles.settingsButton}
              onClick={onOpenSettings}
              aria-label="Profile settings"
              title="Profile settings"
            >
              <svg className={styles.settingsIcon} viewBox="0 0 24 24">
                <path d="M19.14 12.94c.04-.31.06-.63.06-.94 0-.31-.02-.63-.06-.94l2.03-1.58c.18-.14.23-.41.12-.61l-1.92-3.32c-.12-.22-.37-.29-.59-.22l-2.39.96c-.5-.38-1.03-.7-1.62-.94l-.36-2.54c-.04-.24-.24-.41-.48-.41h-3.84c-.24 0-.43.17-.47.41l-.36 2.54c-.59.24-1.13.57-1.62.94l-2.39-.96c-.22-.08-.47 0-.59.22L2.74 8.87c-.12.21-.08.47.12.61l2.03 1.58c-.04.31-.06.63-.06.94s.02.63.06.94l-2.03 1.58c-.18.14-.23.41-.12.61l1.92 3.32c.12.22.37.29.59.22l2.39-.96c.5.38 1.03.7 1.62.94l.36 2.54c.05.24.24.41.48.41h3.84c.24 0 .44-.17.47-.41l.36-2.54c.59-.24 1.13-.56 1.62-.94l2.39.96c.22.08.47 0 .59-.22l1.92-3.32c.12-.22.07-.47-.12-.61l-2.01-1.58zM12 15.6c-1.98 0-3.6-1.62-3.6-3.6s1.62-3.6 3.6-3.6 3.6 1.62 3.6 3.6-1.62 3.6-3.6 3.6z" />
              </svg>
            </button>
          )}
          {onSignOut && (
            <button
              className={styles.signOutButton}
              onClick={handleSignOut}
              aria-label="Sign out"
              title="Sign out"
            >
              <svg className={styles.signOutIcon} viewBox="0 0 24 24">
                <path d="M17 7l-1.41 1.41L18.17 11H8v2h10.17l-2.58 2.58L17 17l5-5zM4 5h8V3H4c-1.1 0-2 .9-2 2v14c0 1.1.9 2 2 2h8v-2H4V5z" />
              </svg>
            </button>
          )}
          {messages.length > 0 && onClear && (
            <button
              className={styles.clearButton}
              onClick={onClear}
              aria-label="Clear chat"
              title="Clear chat history"
            >
              <svg className={styles.clearIcon} viewBox="0 0 24 24">
                <path d="M6 19c0 1.1.9 2 2 2h8c1.1 0 2-.9 2-2V7H6v12zM19 4h-3.5l-1-1h-5l-1 1H5v2h14V4z" />
              </svg>
            </button>
          )}
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close chat"
          >
            <svg className={styles.closeIcon} viewBox="0 0 24 24">
              <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
            </svg>
          </button>
        </div>
      </div>

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {messages.length === 0 && !isLoading && (
          <EmptyState />
        )}

        {messages.map((message, index) => (
          <Message key={index} message={message} />
        ))}

        {isLoading && <LoadingIndicator />}

        {error && (
          <div className={styles.errorMessage}>
            <span>{error}</span>
            {onRetry && (
              <button className={styles.retryButton} onClick={onRetry}>
                Retry
              </button>
            )}
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <ChatInput onSend={onSend} disabled={isLoading} autoFocus={true} />
    </div>
  );
}

/**
 * Loading indicator with animated dots.
 */
function LoadingIndicator() {
  return (
    <div className={styles.loadingIndicator}>
      <div className={styles.loadingDots}>
        <div className={styles.loadingDot} />
        <div className={styles.loadingDot} />
        <div className={styles.loadingDot} />
      </div>
    </div>
  );
}

/**
 * Empty state when no messages.
 */
function EmptyState() {
  return (
    <div className={styles.emptyState}>
      <svg className={styles.emptyStateIcon} viewBox="0 0 24 24">
        <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
      </svg>
      <div className={styles.emptyStateTitle}>Ask me anything!</div>
      <div className={styles.emptyStateText}>
        I can help you understand concepts from the Physical AI & Humanoid Robotics book.
      </div>
    </div>
  );
}
