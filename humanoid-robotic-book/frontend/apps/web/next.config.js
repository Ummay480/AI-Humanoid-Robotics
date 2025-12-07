/** @type {import('next').NextConfig} */
const nextConfig = {
  reactStrictMode: true,
  images: {
    remotePatterns: [
      {
        protocol: 'https',
        hostname: '**',
      },
    ],
  },
  experimental: {
    optimizePackageImports: ['@humanoid-robotics/ui'],
  },
  webpack: (config, { isServer }) => {
    return config;
  },
};

module.exports = nextConfig;
