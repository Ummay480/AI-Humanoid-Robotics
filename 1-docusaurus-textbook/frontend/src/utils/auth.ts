/**
 * Utility functions for authentication headers
 */

import { useAuth } from '../contexts/AuthContext';

/**
 * Get authentication headers with token if available
 * @returns Promise with headers object
 */
export async function getAuthHeaders(): Promise<Record<string, string>> {
  // Since we can't use hooks outside of React components,
  // we'll implement a simple token-based approach
  // In a real implementation, you'd use the auth context properly

  const headers: Record<string, string> = {};

  // Get token from localStorage or sessionStorage
  const token = localStorage.getItem('auth-token') || sessionStorage.getItem('auth-token');

  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  return headers;
}

/**
 * Set authentication token
 * @param token - JWT token to store
 * @param remember - Whether to store in localStorage (persistent) or sessionStorage (session only)
 */
export function setAuthToken(token: string, remember: boolean = true): void {
  if (remember) {
    localStorage.setItem('auth-token', token);
  } else {
    sessionStorage.setItem('auth-token', token);
  }
}

/**
 * Remove authentication token
 */
export function removeAuthToken(): void {
  localStorage.removeItem('auth-token');
  sessionStorage.removeItem('auth-token');
}

/**
 * Get current user info from token (decode JWT without verification)
 * @returns User info object or null if no token or invalid
 */
export function getCurrentUserFromToken(): any | null {
  const token = localStorage.getItem('auth-token') || sessionStorage.getItem('auth-token');

  if (!token) {
    return null;
  }

  try {
    // Split token and decode payload (second part)
    const parts = token.split('.');
    if (parts.length !== 3) {
      return null;
    }

    const payload = parts[1];
    // Add padding if needed
    const paddedPayload = payload + '='.repeat((4 - payload.length % 4) % 4);
    const decoded = atob(paddedPayload);

    return JSON.parse(decoded);
  } catch (error) {
    console.error('Error decoding token:', error);
    return null;
  }
}