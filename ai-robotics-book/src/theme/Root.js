import React, { useState, useCallback, useEffect } from 'react';
import AuthProvider from '../components/AuthProvider';
import ChatWidget from '../components/ChatWidget';
import SelectionHandler from '../components/SelectionHandler';
import AuthModal from '../components/AuthModal';
import NavbarLoginButton from '../components/NavbarLoginButton';

/**
 * Custom Root component that wraps the entire Docusaurus app.
 * Provides authentication context and adds ChatWidget globally.
 */
export default function Root({ children }) {
  const [pendingSelection, setPendingSelection] = useState(null);
  const [showAuthModal, setShowAuthModal] = useState(false);

  const handleAskAboutSelection = useCallback((selectedText) => {
    setPendingSelection(selectedText);
  }, []);

  const clearPendingSelection = useCallback(() => {
    setPendingSelection(null);
  }, []);

  // Listen for global auth modal open events (from navbar button)
  useEffect(() => {
    const handleOpenAuthModal = () => {
      setShowAuthModal(true);
    };

    window.addEventListener('openAuthModal', handleOpenAuthModal);
    return () => {
      window.removeEventListener('openAuthModal', handleOpenAuthModal);
    };
  }, []);

  const handleAuthSuccess = () => {
    setShowAuthModal(false);
  };

  const handleCloseAuthModal = () => {
    setShowAuthModal(false);
  };

  return (
    <AuthProvider>
      <SelectionHandler onAskAboutSelection={handleAskAboutSelection}>
        {children}
        <ChatWidget
          pendingSelection={pendingSelection}
          onSelectionHandled={clearPendingSelection}
        />
        {/* Navbar login button (renders via portal) */}
        <NavbarLoginButton />
        {/* Global auth modal triggered by navbar button */}
        <AuthModal
          isOpen={showAuthModal}
          onClose={handleCloseAuthModal}
          onSuccess={handleAuthSuccess}
        />
      </SelectionHandler>
    </AuthProvider>
  );
}
