class FrameQueue {
                        
                                                            void enqueue(cv::UMat frame) {
                                                            std::unique_lock<std::mutex> lock(mutex);
                                                            frameQueue.push(frame.clone()); // Enqueue a copy
                                                            lock.unlock();
                                                            condition.notify_one();
                                                        }
                                        
                                            cv::UMat dequeue() {
                                                std::unique_lock<std::mutex> lock(mutex);
                                                condition.wait(lock, [this]{ return !frameQueue.empty() || !running; });
                                                if (!running && frameQueue.empty()) {
                                                    return cv::Mat(); // Return an empty Mat if the queue is empty and we are no longer running
                                                }
                                                cv::UMat frame = frameQueue.front();
                                                frameQueue.pop();
                                               return frame;
                                            }
                                        
                                            void stop() {
                                                std::unique_lock<std::mutex> lock(mutex);
                                                running = false;
                                                lock.unlock();
                                                condition.notify_all();
                                            }
                
  