Gem::Specification.new do |s|
  s.name = 'pid_controller_ng'
  s.summary = "WIP"
  s.description = "WIP"
  s.authors = ["Rick Hull"]
  s.homepage = "https://github.com/rickhull/pid_controller_ng"
  s.license = "LGPL-3.0"

  s.required_ruby_version = "> 2"

  s.version = File.read(File.join(__dir__, 'VERSION')).chomp

  s.files = %w[pid_controller_ng.gemspec VERSION README.md Rakefile]
  s.files += Dir['lib/**/*.rb']
  s.files += Dir['test/**/*.rb']
  s.files += Dir['demo/**/*.rb']
end
