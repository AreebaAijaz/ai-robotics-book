/**
 * Frontend TypeScript Types for RAG Chatbot
 *
 * These types define the data structures used in the frontend React components
 * and API client. They mirror the API contract but include additional
 * client-side state properties.
 */

// ============================================================================
// API Request/Response Types
// ============================================================================

/**
 * Request body for POST /api/chat
 */
export interface ChatRequest {
  session_id: string;
  message: string;
}

/**
 * Request body for POST /api/chat/selection
 */
export interface SelectionChatRequest {
  session_id: string;
  selected_text: string;
  question?: string;
}

/**
 * Response from both /api/chat and /api/chat/selection
 */
export interface ChatResponse {
  message_id: string;
  content: string;
  citations: Citation[];
  response_time_ms: number;
}

/**
 * Source citation referencing book content
 */
export interface Citation {
  module: string;
  chapter: string;
  section?: string;
  relevance_score?: number;
}

/**
 * Response from GET /api/health
 */
export interface HealthResponse {
  status: 'healthy' | 'degraded' | 'unhealthy';
  timestamp: string;
  dependencies?: {
    qdrant?: DependencyStatus;
    postgres?: DependencyStatus;
    openai?: DependencyStatus;
  };
}

/**
 * Individual dependency health status
 */
export interface DependencyStatus {
  status: 'connected' | 'disconnected' | 'error';
  latency_ms?: number;
  error?: string;
}

/**
 * Error response structure
 */
export interface ErrorResponse {
  error: string;
  message: string;
  details?: Record<string, unknown>;
}

// ============================================================================
// Client-Side State Types
// ============================================================================

/**
 * Message role enum
 */
export type MessageRole = 'user' | 'assistant';

/**
 * Individual chat message in the conversation
 */
export interface ChatMessage {
  id: string;
  role: MessageRole;
  content: string;
  timestamp: number;
  citations?: Citation[];
  selectedText?: string;
  isLoading?: boolean;
  error?: string;
}

/**
 * Complete chat session state stored in sessionStorage
 */
export interface ChatSessionState {
  sessionId: string;
  messages: ChatMessage[];
  createdAt: number;
}

// ============================================================================
// Component Props Types
// ============================================================================

/**
 * Props for the main ChatWidget component
 */
export interface ChatWidgetProps {
  /** Initial open state of the chat panel */
  defaultOpen?: boolean;
  /** Position of the floating button */
  position?: 'bottom-right' | 'bottom-left';
  /** Custom z-index for layering */
  zIndex?: number;
}

/**
 * Props for the ChatPanel component
 */
export interface ChatPanelProps {
  /** Whether the panel is currently visible */
  isOpen: boolean;
  /** Callback when user requests to close the panel */
  onClose: () => void;
  /** Current chat messages */
  messages: ChatMessage[];
  /** Callback when user sends a message */
  onSendMessage: (message: string) => void;
  /** Whether a message is currently being processed */
  isLoading: boolean;
}

/**
 * Props for the ChatInput component
 */
export interface ChatInputProps {
  /** Callback when user submits a message */
  onSubmit: (message: string) => void;
  /** Whether input should be disabled (e.g., during loading) */
  disabled?: boolean;
  /** Placeholder text */
  placeholder?: string;
  /** Pre-filled value (e.g., from text selection) */
  initialValue?: string;
}

/**
 * Props for the SelectionHandler component
 */
export interface SelectionHandlerProps {
  /** Callback when user clicks "Ask about this" */
  onAskAboutSelection: (selectedText: string) => void;
  /** Whether to enable selection detection */
  enabled?: boolean;
  /** Content area selector to limit selection detection */
  contentSelector?: string;
}

/**
 * Props for the AskButton component (appears on text selection)
 */
export interface AskButtonProps {
  /** Position to render the button */
  position: { top: number; left: number };
  /** Callback when button is clicked */
  onClick: () => void;
  /** Whether button is visible */
  visible: boolean;
}

/**
 * Props for individual message rendering
 */
export interface MessageProps {
  message: ChatMessage;
  /** Whether to show citation links */
  showCitations?: boolean;
}

// ============================================================================
// API Client Types
// ============================================================================

/**
 * Configuration for the API client
 */
export interface ApiClientConfig {
  /** Base URL for the API (default: http://localhost:8000) */
  baseUrl?: string;
  /** Request timeout in milliseconds (default: 30000) */
  timeout?: number;
}

/**
 * Result type for API calls (success or error)
 */
export type ApiResult<T> =
  | { success: true; data: T }
  | { success: false; error: ErrorResponse };

// ============================================================================
// Hook Return Types
// ============================================================================

/**
 * Return type for the useChat hook
 */
export interface UseChatReturn {
  /** Current session ID */
  sessionId: string;
  /** Chat message history */
  messages: ChatMessage[];
  /** Whether a request is in progress */
  isLoading: boolean;
  /** Current error (if any) */
  error: string | null;
  /** Send a general chat message */
  sendMessage: (message: string) => Promise<void>;
  /** Send a selection-based message */
  sendSelectionMessage: (selectedText: string, question?: string) => Promise<void>;
  /** Clear chat history and start new session */
  clearChat: () => void;
}

/**
 * Return type for the useTextSelection hook
 */
export interface UseTextSelectionReturn {
  /** Currently selected text (empty string if none) */
  selectedText: string;
  /** Position for the "Ask" button */
  buttonPosition: { top: number; left: number } | null;
  /** Clear the current selection */
  clearSelection: () => void;
}

// ============================================================================
// Storage Keys
// ============================================================================

/**
 * sessionStorage key for chat session data
 */
export const STORAGE_KEY = 'chatbot_session' as const;

// ============================================================================
// Validation Constants
// ============================================================================

export const VALIDATION = {
  /** Maximum length for user messages */
  MAX_MESSAGE_LENGTH: 2000,
  /** Maximum length for selected text */
  MAX_SELECTION_LENGTH: 2000,
  /** Minimum length for selected text to trigger button */
  MIN_SELECTION_LENGTH: 3,
  /** Maximum length for optional question on selection */
  MAX_QUESTION_LENGTH: 500,
} as const;
