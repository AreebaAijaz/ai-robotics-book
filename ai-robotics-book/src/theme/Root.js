import React, { useState, useCallback } from 'react';
import ChatWidget from '../components/ChatWidget';
import SelectionHandler from '../components/SelectionHandler';

/**
 * Custom Root component that wraps the entire Docusaurus app.
 * This allows us to add the ChatWidget globally on all pages
 * and connect the SelectionHandler to trigger selection queries.
 */
export default function Root({ children }) {
  const [pendingSelection, setPendingSelection] = useState(null);

  const handleAskAboutSelection = useCallback((selectedText) => {
    setPendingSelection(selectedText);
  }, []);

  const clearPendingSelection = useCallback(() => {
    setPendingSelection(null);
  }, []);

  return (
    <SelectionHandler onAskAboutSelection={handleAskAboutSelection}>
      {children}
      <ChatWidget
        pendingSelection={pendingSelection}
        onSelectionHandled={clearPendingSelection}
      />
    </SelectionHandler>
  );
}
