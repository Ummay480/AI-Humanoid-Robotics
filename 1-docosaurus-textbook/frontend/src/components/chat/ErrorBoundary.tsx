import React, { Component, ErrorInfo, ReactNode } from 'react';

/**
 * ErrorBoundary Props
 */
interface ErrorBoundaryProps {
  children: ReactNode;
}

/**
 * ErrorBoundary State
 */
interface ErrorBoundaryState {
  hasError: boolean;
  error: Error | null;
}

/**
 * ErrorBoundary Component - Catches React errors in chat components
 *
 * Provides graceful fallback UI when chat components encounter errors.
 * Logs errors to console for debugging.
 *
 * @example
 * ```tsx
 * <ErrorBoundary>
 *   <ChatWidget />
 * </ErrorBoundary>
 * ```
 */
export class ErrorBoundary extends Component<ErrorBoundaryProps, ErrorBoundaryState> {
  constructor(props: ErrorBoundaryProps) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
    };
  }

  static getDerivedStateFromError(error: Error): ErrorBoundaryState {
    return {
      hasError: true,
      error,
    };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    console.error('[ErrorBoundary] Chat error caught:', error);
    console.error('[ErrorBoundary] Error info:', errorInfo);
  }

  handleReset = () => {
    this.setState({
      hasError: false,
      error: null,
    });
    // Reload page to reset state
    window.location.reload();
  };

  render() {
    if (this.state.hasError) {
      return (
        <div
          style={{
            position: 'fixed',
            bottom: '2rem',
            right: '2rem',
            background: 'var(--ifm-color-danger)',
            color: 'white',
            padding: '1rem 1.5rem',
            borderRadius: '8px',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
            zIndex: 1000,
            maxWidth: '300px',
          }}
        >
          <div style={{ fontWeight: 600, marginBottom: '0.5rem' }}>
            Chat Unavailable
          </div>
          <div style={{ fontSize: '0.9rem', marginBottom: '1rem' }}>
            {this.state.error?.message || 'An error occurred'}
          </div>
          <button
            onClick={this.handleReset}
            style={{
              background: 'white',
              color: 'var(--ifm-color-danger)',
              border: 'none',
              padding: '0.5rem 1rem',
              borderRadius: '4px',
              cursor: 'pointer',
              fontWeight: 600,
            }}
          >
            Reload Page
          </button>
        </div>
      );
    }

    return this.props.children;
  }
}
