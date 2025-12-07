export interface Chapter {
  id: string;
  number: number;
  name: string;
  description: string;
  slug: string;
  lessonsCount: number;
  lessons: Lesson[];
}

export interface Lesson {
  id: string;
  number: number;
  chapterId: string;
  title: string;
  slug: string;
  content: string; // Markdown content
  sections: Section[];
  learningObjectives: string[];
}

export interface Section {
  id: string;
  title: string;
  level: number;
  content: string;
  subsections: Section[];
}

export interface TableOfContents {
  chapters: Chapter[];
  currentChapterId?: string;
  currentLessonId?: string;
}
