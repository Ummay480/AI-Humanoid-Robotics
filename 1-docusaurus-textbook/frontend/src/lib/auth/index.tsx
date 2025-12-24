import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.REACT_APP_AUTH_BASE_URL || "http://localhost:8000/auth",
  fetchOptions: {
    credentials: "include",
  },
});