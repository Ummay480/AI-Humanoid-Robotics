import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';

export default function AppWrapper({children}) {
  return <AuthProvider>{children}</AuthProvider>;
}