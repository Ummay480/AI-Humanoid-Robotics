export default function Home() {
  return (
    <main className="min-h-screen bg-gradient-to-b from-slate-50 to-white dark:from-slate-950 dark:to-slate-900">
      <div className="max-w-6xl mx-auto px-4 py-12 md:py-20">
        {/* Hero Section */}
        <section className="text-center mb-16">
          <h1 className="text-4xl md:text-5xl font-bold text-slate-900 dark:text-white mb-4">
            Physical AI & Humanoid Robotics
          </h1>
          <p className="text-xl md:text-2xl text-slate-600 dark:text-slate-300 mb-8">
            An AI-Native Interactive Textbook
          </p>
          <p className="text-lg text-slate-500 dark:text-slate-400 max-w-2xl mx-auto mb-12">
            Learn comprehensive robotics from fundamentals to cutting-edge AI techniques.
            150 lessons across 10 chapters covering kinematics, dynamics, control, vision, and more.
          </p>

          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <a
              href="/chapters"
              className="inline-block bg-blue-600 hover:bg-blue-700 text-white font-semibold px-8 py-3 rounded-lg transition-colors"
            >
              Start Learning
            </a>
            <a
              href="/search"
              className="inline-block bg-slate-200 hover:bg-slate-300 dark:bg-slate-700 dark:hover:bg-slate-600 text-slate-900 dark:text-white font-semibold px-8 py-3 rounded-lg transition-colors"
            >
              Search Content
            </a>
          </div>
        </section>

        {/* Features Section */}
        <section className="grid md:grid-cols-3 gap-8 mb-16">
          <div className="bg-white dark:bg-slate-800 rounded-lg p-6 shadow-sm">
            <h3 className="text-xl font-semibold text-slate-900 dark:text-white mb-3">
              150 Comprehensive Lessons
            </h3>
            <p className="text-slate-600 dark:text-slate-300">
              15 lessons per chapter covering theory, implementation, and practical applications
            </p>
          </div>

          <div className="bg-white dark:bg-slate-800 rounded-lg p-6 shadow-sm">
            <h3 className="text-xl font-semibold text-slate-900 dark:text-white mb-3">
              AI-Powered Chat
            </h3>
            <p className="text-slate-600 dark:text-slate-300">
              Ask questions about content with intelligent chatbot assistance powered by OpenAI
            </p>
          </div>

          <div className="bg-white dark:bg-slate-800 rounded-lg p-6 shadow-sm">
            <h3 className="text-xl font-semibold text-slate-900 dark:text-white mb-3">
              Fully Accessible
            </h3>
            <p className="text-slate-600 dark:text-slate-300">
              WCAG 2.1 AA compliant with keyboard navigation and screen reader support
            </p>
          </div>
        </section>

        {/* Chapters Preview */}
        <section className="mb-16">
          <h2 className="text-3xl font-bold text-slate-900 dark:text-white mb-8">
            10 Comprehensive Chapters
          </h2>
          <div className="grid md:grid-cols-2 gap-4">
            {chapters.map((chapter) => (
              <div
                key={chapter.number}
                className="bg-white dark:bg-slate-800 rounded-lg p-6 shadow-sm hover:shadow-md transition-shadow"
              >
                <h3 className="text-lg font-semibold text-slate-900 dark:text-white mb-2">
                  Chapter {chapter.number}: {chapter.name}
                </h3>
                <p className="text-slate-600 dark:text-slate-300 text-sm">
                  {chapter.description}
                </p>
              </div>
            ))}
          </div>
        </section>

        {/* CTA Section */}
        <section className="bg-blue-600 text-white rounded-lg p-8 text-center">
          <h2 className="text-3xl font-bold mb-4">Ready to Learn?</h2>
          <p className="text-lg mb-6 opacity-90">
            Begin your journey into humanoid robotics and physical AI today.
          </p>
          <a
            href="/chapters"
            className="inline-block bg-white text-blue-600 font-semibold px-8 py-3 rounded-lg hover:bg-slate-100 transition-colors"
          >
            Explore the Textbook
          </a>
        </section>
      </div>
    </main>
  );
}

const chapters = [
  {
    number: 1,
    name: 'Fundamentals',
    description: 'Introduction to robotics, hardware, and software basics',
  },
  {
    number: 2,
    name: 'Kinematics',
    description: 'Motion analysis, trajectory planning, and forward/inverse kinematics',
  },
  {
    number: 3,
    name: 'Dynamics',
    description: 'Physics-based simulation, forces, and dynamics modeling',
  },
  {
    number: 4,
    name: 'Control',
    description: 'Feedback control, PID, LQR, and AI-based control systems',
  },
  {
    number: 5,
    name: 'Vision',
    description: 'Computer vision, image processing, and perception',
  },
  {
    number: 6,
    name: 'Motion Planning',
    description: 'Path planning, RRT, trajectory optimization',
  },
  {
    number: 7,
    name: 'Manipulation',
    description: 'Grasping, manipulation, and dexterous control',
  },
  {
    number: 8,
    name: 'Locomotion',
    description: 'Walking, running, balance, and gaits',
  },
  {
    number: 9,
    name: 'HRI',
    description: 'Human-robot interaction and learning from humans',
  },
  {
    number: 10,
    name: 'AI',
    description: 'Machine learning, deep learning, and AI for robotics',
  },
];
