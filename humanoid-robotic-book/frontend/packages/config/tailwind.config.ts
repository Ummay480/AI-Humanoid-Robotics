import type { Config } from 'tailwindcss';
import designTokens from './design-tokens';

const config: Config = {
  content: [
    '../../apps/web/app/**/*.{js,ts,jsx,tsx}',
    '../../apps/web/components/**/*.{js,ts,jsx,tsx}',
    '../../packages/ui/**/*.{js,ts,jsx,tsx}',
  ],
  theme: {
    extend: {
      colors: {
        primary: designTokens.colors.primary,
        secondary: designTokens.colors.secondary,
        success: designTokens.colors.success,
        warning: designTokens.colors.warning,
        error: designTokens.colors.error,
      },
      fontFamily: {
        sans: designTokens.typography.fontFamily.sans,
        mono: designTokens.typography.fontFamily.mono,
      },
      fontSize: designTokens.typography.fontSize,
      spacing: designTokens.spacing,
      breakpoints: designTokens.breakpoints,
    },
  },
  plugins: [],
};

export default config;
