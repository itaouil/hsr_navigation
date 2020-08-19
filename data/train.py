from torchvision import transforms
from detecto import core, utils, visualize

augmentations = transforms.Compose([
    transforms.ToPILImage(),
    transforms.RandomHorizontalFlip(0.5),
    transforms.ColorJitter(saturation=0.5),
    transforms.ToTensor(),
    utils.normalize_transform(),
])

dataset = core.Dataset('images/', transform=augmentations)

loader = core.DataLoader(dataset, batch_size=2, shuffle=True)

print("Finished data augmentation...")

model = core.Model.load('hsr_model.pth', ['push', 'kick', 'grasp'])

model.fit(loader, None, epochs=20, learning_rate=0.001, lr_step_size=5, verbose=True)
               
model.save('hsr_weights_1.pth')

print("Finished training process...")