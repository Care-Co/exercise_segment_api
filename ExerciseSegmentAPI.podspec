Pod::Spec.new do |spec|
  spec.name         = "ExerciseSegmentAPI"
  spec.version      = "2.2.3"
  spec.summary      = "Exercise Segment Analysis API v2.2.3"
  spec.description  = <<-DESC
                     A C API for analyzing exercise progress and providing real-time feedback based on Google ML Kit pose data.
                     
                     Features:
                     - 33 landmark pose analysis
                     - Real-time progress tracking (60fps)
                     - JSON-based workout storage
                     - User role separation (Recorder/User)
                     - Smart pose analysis with user position adaptation
                     - Enhanced segment management
                     - Swift integration support with type safety
                     - Exercise mode with ankle-centered positioning
                     - Shoulder-width based scaling for exercise mode
                     DESC

  spec.homepage     = "https://github.com/juntaenoh/exercise_segment_api"
  spec.license      = { :type => "MIT", :file => "LICENSE" }
  spec.author       = { "Exercise Segment API Team" => "contact@exercisesegmentapi.com" }

  spec.platform     = :ios, "16.0"
  spec.source       = { :git => "https://github.com/juntaenoh/exercise_segment_api.git", :tag => "#{spec.version}" }

  spec.source_files = [
    "include/**/*.h",
    "src/**/*.c"
  ]

  spec.public_header_files = "include/**/*.h"
  
  spec.requires_arc = false
  
  spec.pod_target_xcconfig = {
    'HEADER_SEARCH_PATHS' => '$(PODS_TARGET_SRCROOT)/include'
  }
  
  spec.user_target_xcconfig = {
    'HEADER_SEARCH_PATHS' => '$(PODS_TARGET_SRCROOT)/include'
  }
end
